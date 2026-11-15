"""Integrated freeway simulation: mainline + metered ramp + CVs + braking disturbance.

Per time step (dt = 0.1 s):
  1. arrivals (mainline and ramp, seeded Bernoulli)
  2. mainline admission and ramp release (gap acceptance from ramp_simulation.py)
  3. accelerations: human IDM, ACC (connected vehicles), prescribed disturbance
  4. position/speed update, detectors, exits, overlap check
Every detector interval (30 s) the TMC loop runs:
  observe -> estimate -> forecast -> decide (ramp meter rate) -> act (next steps)

Usage examples
  python run_simulation.py --quick                        # fast smoke test
  python run_simulation.py                                # 0/20/50/70 % CV, seed 1
  python run_simulation.py --seeds 1 2 3 4 5 --ramp-control fixed alinea
Then:  python analysis.py

Reused code (attributed): gap-acceptance and entry logic come from
ramp_simulation.py; detectors from detectors.py; IDM from idm.py.
"""
import argparse
import csv
import json
import random
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import ramp_simulation as rs
from idm import Vehicle, idm_accel, follower_accel, step_vehicle
from detectors import create_detectors
from observations import (ObservationDefects, ObservationPipeline, StateEstimator,
                          ForecastManager, attach_truth, forecast_mae)
from controller import (HDV_PARAMS, CV_PARAMS, ACCConfig, ACCController,
                        make_ramp_controller)

MILE = 1609.344
V_FREE_MPS = 30.0           # reference speed for delay accounting
HARD_BRAKE_MPS2 = -3.0      # threshold for counting hard-braking vehicle-steps
AFFECTED_DIP_MPS = 1.0      # a follower is "affected" if its speed dips more than this
N_TRACKED = 60              # followers tracked behind the disturbance vehicle


@dataclass
class Scenario:
    scenario_id: str = ""
    seed: int = 1
    dt: float = 0.1
    duration_s: float = 1200.0
    warmup_s: float = 300.0             # metrics are measured after this time

    mainline_demand_vph: float = 1500.0
    ramp_demand_vph: float = 400.0

    cv_penetration: float = 0.0
    ramp_control: str = "fixed"         # none | fixed | alinea
    fixed_meter_vph: float = 600.0
    meter_sensor: str = "D4"
    # How fast a ramp vehicle is when it tries to merge:
    #   "match_leader": speed of the mainline vehicle just ahead (approximates a
    #                   long acceleration lane where the driver has matched speed)
    #   "fixed":        rs.RAMP_ENTRY_SPEED_MPS (20 m/s) as in ramp_simulation.py
    ramp_entry_mode: str = "match_leader"

    disturbance: bool = True
    disturbance_time_s: float = 600.0
    disturbance_target_m: float = 2.0 * MILE
    brake_accel_mps2: float = -2.0      # same profile as platoon_experiment.py
    brake_duration_s: float = 3.0

    # degraded-information / degraded-vehicle conditions
    sensor_dropout_prob: float = 0.0    # ACC radar dropout (per update)
    actuator_lag_s: float = 0.3
    obs_missing_prob: float = 0.0       # detector report lost
    obs_delay_intervals: int = 0
    obs_count_noise_sd: float = 0.0

    save_trajectories: bool = False
    traj_interval_s: float = 1.0


def make_scenario_id(sc):
    sid = f"{sc.ramp_control}_cv{round(100 * sc.cv_penetration):02d}"
    if sc.sensor_dropout_prob > 0:
        sid += f"_drop{round(100 * sc.sensor_dropout_prob):02d}"
    if sc.obs_missing_prob > 0 or sc.obs_delay_intervals > 0 or sc.obs_count_noise_sd > 0:
        sid += "_obsdefect"
    if not sc.disturbance:
        sid += "_nodist"
    return sid


def write_csv(path, rows, fieldnames=None):
    if not rows:
        return
    fieldnames = fieldnames or list(rows[0])
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


# ---------------------------------------------------------------------------
# One simulation run
# ---------------------------------------------------------------------------
def run_scenario(sc, out_root="results", verbose=True):
    if not sc.scenario_id:
        sc.scenario_id = make_scenario_id(sc)
    out_dir = Path(out_root) / sc.scenario_id / f"seed_{sc.seed}"
    out_dir.mkdir(parents=True, exist_ok=True)
    wall_start = time.time()

    dt = sc.dt
    n_steps = round(sc.duration_s / dt)
    s = sc.seed

    # Independent random streams: the arrival sequence does NOT change when the
    # CV penetration or controller changes, so scenarios are paired by seed.
    arr_main = random.Random(f"arrivals-main-{s}")
    arr_ramp = random.Random(f"arrivals-ramp-{s}")
    tag_main = random.Random(f"cv-tag-main-{s}")
    tag_ramp = random.Random(f"cv-tag-ramp-{s}")
    rng_sensor = random.Random(f"acc-sensor-{s}")
    rng_obs = random.Random(f"obs-defects-{s}")
    p_main = sc.mainline_demand_vph / 3600.0 * dt
    p_ramp = sc.ramp_demand_vph / 3600.0 * dt

    # Components
    detectors = create_detectors()
    sensor_ids = [f"D{i + 1}" for i in range(len(detectors))]
    interval_s = detectors[0].interval_s
    interval_steps = round(interval_s / dt)
    pipeline = ObservationPipeline(
        ObservationDefects(sc.obs_missing_prob, sc.obs_delay_intervals,
                           sc.obs_count_noise_sd), rng_obs)
    estimator = StateEstimator(sensor_ids, interval_s)
    forecaster = ForecastManager(sensor_ids, interval_s, horizon_intervals=2)
    ramp_ctrl = make_ramp_controller(sc.ramp_control, sc.fixed_meter_vph,
                                     sc.meter_sensor)
    acc = ACCController(ACCConfig(actuator_lag_s=sc.actuator_lag_s,
                                  sensor_dropout_prob=sc.sensor_dropout_prob))
    stats = rs.SimulationStats()

    # State
    vehicles, meta = [], {}
    queue_main, queue_ramp = [], []
    next_id = 0
    meter_rate = None if sc.ramp_control == "none" else (
        sc.fixed_meter_vph if sc.ramp_control == "fixed" else ramp_ctrl.rate)
    meter_credit = 1.0
    cv_ids = set()

    # Logs
    obs_log, est_log, fc_log, ctrl_log = [], [], [], []
    queue_log, traj_log = [], []
    min_gap = float("inf")
    min_accel, hard_brake_steps = 0.0, 0
    exits_in_window = 0
    ramp_queue_sum, ramp_queue_peak, main_queue_peak = 0, 0, 0
    overlap_error = False

    # Disturbance bookkeeping
    dist_id, dist_t0 = None, None
    tracked, tracked_pre, tracked_series, rank_of = [], {}, {}, {}
    disturbance_info = {
        "scenario_id": sc.scenario_id, "seed": s,
        "vehicle_id": None, "start_time_s": None,
        "start_position_m": None, "start_speed_mps": None,
        "target_position_m": sc.disturbance_target_m,
        "requested_brake_mps2": sc.brake_accel_mps2,
        "actual_min_brake_mps2": None, "braking_steps": 0,
    }

    traj_steps = max(1, round(sc.traj_interval_s / dt))
    ramp_x = rs.RAMP_POSITION_M

    try:
        for step in range(n_steps):
            t = step * dt

            # ---- 1. arrivals --------------------------------------------
            if arr_main.random() < p_main:
                queue_main.append({"t_req": t, "u": tag_main.random()})
                stats.mainline_requested += 1
            if arr_ramp.random() < p_ramp:
                queue_ramp.append({"t_req": t, "u": tag_ramp.random()})
                stats.ramp_requested += 1

            # ---- 2. mainline admission and ramp release -----------------
            if queue_main:
                ent = queue_main[0]
                is_cv = ent["u"] < sc.cv_penetration
                veh = rs.create_mainline_vehicle(next_id, vehicles)
                veh.params = CV_PARAMS if is_cv else HDV_PARAMS
                if rs.can_enter_mainline(vehicles, veh, stats):
                    vehicles.append(veh)
                    queue_main.pop(0)
                    stats.mainline_admitted += 1
                    meta[next_id] = {"t_req": ent["t_req"], "x0": 0.0,
                                     "is_cv": is_cv, "t_exit": None}
                    if is_cv:
                        cv_ids.add(next_id)
                    next_id += 1

            if meter_rate is None:
                meter_credit = 1.0
            else:
                meter_credit = min(1.0, meter_credit + meter_rate * dt / 3600.0)

            if queue_ramp:
                if meter_credit >= 1.0:
                    ent = queue_ramp[0]
                    is_cv = ent["u"] < sc.cv_penetration
                    entry_speed = rs.RAMP_ENTRY_SPEED_MPS
                    if sc.ramp_entry_mode == "match_leader":
                        ahead = [c for c in vehicles if c.position_m > ramp_x]
                        if ahead:
                            entry_speed = min(ahead, key=lambda c: c.position_m).speed_mps
                    veh = Vehicle(next_id, ramp_x, entry_speed,
                                  CV_PARAMS if is_cv else HDV_PARAMS)
                    if rs.check_ramp_merge(vehicles, veh, stats):
                        vehicles.append(veh)
                        queue_ramp.pop(0)
                        stats.ramp_merged += 1
                        meta[next_id] = {"t_req": ent["t_req"], "x0": ramp_x,
                                         "is_cv": is_cv, "t_exit": None}
                        if is_cv:
                            cv_ids.add(next_id)
                        next_id += 1
                        if meter_rate is not None:
                            meter_credit -= 1.0
                else:
                    stats.ramp_meter_waiting += 1

            vehicles.sort(key=lambda c: c.position_m, reverse=True)

            # ---- disturbance vehicle selection ---------------------------
            if sc.disturbance and dist_id is None and t >= sc.disturbance_time_s:
                cands = [c for c in vehicles
                         if c.vehicle_id not in cv_ids and c.position_m < ramp_x]
                if cands:
                    d = min(cands, key=lambda c: abs(c.position_m - sc.disturbance_target_m))
                    dist_id, dist_t0 = d.vehicle_id, t
                    disturbance_info.update(
                        vehicle_id=dist_id, start_time_s=t,
                        start_position_m=d.position_m,
                        start_speed_mps=d.speed_mps)
                    behind = [c for c in vehicles if c.position_m < d.position_m]
                    behind.sort(key=lambda c: c.position_m, reverse=True)
                    tracked = [c.vehicle_id for c in behind[:N_TRACKED]]
                    for r, c in enumerate(behind[:N_TRACKED], start=1):
                        rank_of[c.vehicle_id] = r
                        tracked_pre[c.vehicle_id] = c.speed_mps
                        tracked_series[c.vehicle_id] = []

            # ---- 3. accelerations (all from the same time-t state) -------
            accels = []
            for i, car in enumerate(vehicles):
                leader = vehicles[i - 1] if i > 0 else None
                if car.vehicle_id in cv_ids:
                    a = acc.compute(car, leader, t, dt, rng_sensor)
                else:
                    a = (idm_accel(car.speed_mps, None, 0.0, car.params)
                         if leader is None else follower_accel(car, leader))
                if (car.vehicle_id == dist_id
                        and dist_t0 <= t < dist_t0 + sc.brake_duration_s):
                    a = min(a, sc.brake_accel_mps2)   # braking is a floor: never softer than IDM
                    disturbance_info["braking_steps"] += 1
                    old = disturbance_info["actual_min_brake_mps2"]
                    if old is None or a < old:
                        disturbance_info["actual_min_brake_mps2"] = a
                accels.append(a)

            previous_positions = {c.vehicle_id: c.position_m for c in vehicles}
            for car, a in zip(vehicles, accels):
                step_vehicle(car, a, dt)
                if a < min_accel:
                    min_accel = a
                if a < HARD_BRAKE_MPS2:
                    hard_brake_steps += 1

            now = (step + 1) * dt

            # ---- 4. detectors ---------------------------------------------
            for det in detectors:
                det.update(previous_positions, vehicles, dt)

            if (step + 1) % interval_steps == 0:
                for sid, det in zip(sensor_ids, detectors):
                    det.record_interval(now)
                    pipeline.ingest(sid, det.records[-1], interval_s)
                observations = pipeline.collect(now)
                obs_log.extend(observations)
                estimates = estimator.update(observations, now)
                est_log.extend(estimates.values())
                fc_log.extend(forecaster.update(observations, now))
                cmd = ramp_ctrl.decide(now, estimates, len(queue_ramp))
                meter_rate = cmd["value"]       # applied from the next step on
                ctrl_log.append(cmd)

            # ---- exits and bookkeeping -------------------------------------
            for car in vehicles:
                if car.position_m >= rs.ROAD_LENGTH_M:
                    meta[car.vehicle_id]["t_exit"] = now
                    if now > sc.warmup_s:
                        exits_in_window += 1
                    acc.forget(car.vehicle_id)
            # Check gaps BEFORE removing exiting vehicles, so an overlap at
            # the downstream boundary cannot be hidden by the exit filter.
            for i in range(1, len(vehicles)):
                gap = (vehicles[i - 1].position_m - vehicles[i - 1].params.length
                       - vehicles[i].position_m)
                if gap < min_gap:
                    min_gap = gap
                if gap <= 0:
                    raise RuntimeError(f"Vehicle overlap at t={now:.1f} s "
                                       f"(ids {vehicles[i - 1].vehicle_id}, "
                                       f"{vehicles[i].vehicle_id})")

            before = len(vehicles)
            vehicles = [c for c in vehicles if c.position_m < rs.ROAD_LENGTH_M]
            stats.exits += before - len(vehicles)

            ramp_queue_sum += len(queue_ramp)
            ramp_queue_peak = max(ramp_queue_peak, len(queue_ramp))
            main_queue_peak = max(main_queue_peak, len(queue_main))
            if (step + 1) % round(5.0 / dt) == 0:
                queue_log.append({"time_s": now, "ramp_queue": len(queue_ramp),
                                  "mainline_queue": len(queue_main),
                                  "vehicles_on_road": len(vehicles)})

            if tracked:
                by_id = {c.vehicle_id: c for c in vehicles}
                for vid in tracked:
                    c = by_id.get(vid)
                    if c is not None:
                        tracked_series[vid].append((now, c.speed_mps))

            if sc.save_trajectories and (step + 1) % traj_steps == 0:
                for c in vehicles:
                    traj_log.append({
                        "time_s": round(now, 3), "vehicle_id": c.vehicle_id,
                        "lane_id": c.lane_id, "position_m": round(c.position_m, 3),
                        "speed_mps": round(c.speed_mps, 3),
                        "acceleration_mps2": round(c.accel_mps2, 3),
                        "is_cv": int(c.vehicle_id in cv_ids),
                        "is_disturbance": int(c.vehicle_id == dist_id)})
    except RuntimeError as err:
        overlap_error = True
        print(f"  !! {sc.scenario_id} seed {s}: {err}")

    # ------------------------------------------------------------------
    # Post-run metrics
    # ------------------------------------------------------------------
    t_end = n_steps * dt if not overlap_error else (len(queue_log) * 5.0)
    window_s = max(1.0, sc.duration_s - sc.warmup_s)
    on_road = {c.vehicle_id: c for c in vehicles}

    # Delay accounting: delay_i = (elapsed since request) - (progress / v_free).
    delay_total_s, n_requested = 0.0, 0
    for vid, m in meta.items():
        n_requested += 1
        if m["t_exit"] is not None:
            elapsed, progress = m["t_exit"] - m["t_req"], rs.ROAD_LENGTH_M - m["x0"]
        else:
            c = on_road.get(vid)
            progress = (c.position_m - m["x0"]) if c is not None else 0.0
            elapsed = t_end - m["t_req"]
        delay_total_s += elapsed - progress / V_FREE_MPS
    for ent in queue_main + queue_ramp:          # vehicles still waiting at a boundary
        n_requested += 1
        delay_total_s += t_end - ent["t_req"]

    # Truth for forecast scoring and detector summaries
    truth = {}
    det_rows = []
    for sid, det in zip(sensor_ids, detectors):
        for r in det.records:
            truth[(sid, round(r["time_s"], 6))] = r["flow_veh_h"]
            det_rows.append({"scenario_id": sc.scenario_id, "seed": s, "sensor_id": sid,
                             "name": det.name, "position_m": round(det.position_m, 1),
                             **r})
    attach_truth(fc_log, truth)

    def det_stat(sid, key, fn):
        vals = [r[key] for r in det_rows
                if r["sensor_id"] == sid and r["time_s"] > sc.warmup_s
                and r[key] is not None]
        return fn(vals) if vals else None

    # Disturbance response of tracked followers.
    # A follower is "affected" if its speed dips more than AFFECTED_DIP_MPS below
    # its pre-disturbance speed. Recovery is measured only for affected followers,
    # from the moment of their own lowest speed until speed is back to >= 95 % of
    # the pre-disturbance value for 5 continuous seconds. Followers that leave the
    # road first are censored (counted as unrecovered), not as recovered.
    profile_rows, dips, rec_after_min, rec_abs, max_rank_affected = [], [], [], [], 0
    if dist_id is not None:
        for vid in tracked:
            series = tracked_series[vid]
            pre = tracked_pre[vid]
            after = [(tt, v) for tt, v in series if tt > dist_t0]
            if not after:
                continue
            t_min, v_min = min(after, key=lambda p: p[1])
            dip = pre - v_min
            first_response_s = next(
                (tt - dist_t0 for tt, speed in after
                 if pre - speed > AFFECTED_DIP_MPS), None)
            dips.append(dip)
            rec_t = None
            if dip > AFFECTED_DIP_MPS:
                max_rank_affected = max(max_rank_affected, rank_of[vid])
                thr = 0.95 * pre
                for j, (tt, v) in enumerate(after):
                    if tt < t_min or v < thr:
                        continue
                    win = [(t2, v2) for t2, v2 in after[j:] if t2 <= tt + 5.0 + 1e-9]
                    if win[-1][0] >= tt + 5.0 - 1e-9 and all(v2 >= thr for _, v2 in win):
                        rec_t = tt
                        break
                if rec_t is not None:
                    rec_after_min.append(rec_t - t_min)
                    rec_abs.append(rec_t - dist_t0)
            profile_rows.append({
                "scenario_id": sc.scenario_id, "seed": s, "rank": rank_of[vid],
                "vehicle_id": vid, "is_cv": int(vid in cv_ids),
                "pre_speed_mps": round(pre, 3), "min_speed_mps": round(v_min, 3),
                "dip_mps": round(dip, 3), "t_min_speed_s": round(t_min, 1),
                "affected": int(dip > AFFECTED_DIP_MPS),
                "first_response_s": (None if first_response_s is None
                                     else round(first_response_s, 2)),
                "last_observed_time_s": round(after[-1][0], 2),
                "recovery_after_min_s": None if rec_t is None else round(rec_t - t_min, 1),
                "recovered_at_s": None if rec_t is None else round(rec_t, 1)})

    n_affected = sum(1 for d in dips if d > AFFECTED_DIP_MPS)
    n_recovered = len(rec_after_min)
    n_unrecovered = n_affected - n_recovered
    recovery_fraction = n_recovered / n_affected if n_affected else None

    # Detector-based disturbance response at D3 (2.4 mi, just upstream of the merge).
    # Baseline = mean space-mean speed in the 150 s before the disturbance.
    # Recovery = first time two consecutive intervals are back within 1.5 m/s of baseline.
    d3_dip, d3_recovery = None, None
    if dist_id is not None:
        d3 = [(r["time_s"], r["space_mean_speed_mps"]) for r in det_rows
              if r["sensor_id"] == "D3" and r["space_mean_speed_mps"] is not None]
        pre = [v for tt, v in d3 if dist_t0 - 150.0 <= tt <= dist_t0]
        post = [(tt, v) for tt, v in d3 if tt > dist_t0]
        if pre and post:
            base = sum(pre) / len(pre)
            t_low, v_low = min(post, key=lambda p: p[1])
            d3_dip = base - v_low
            if d3_dip > 1.5:
                later = [(tt, v) for tt, v in post if tt >= t_low]
                for j in range(len(later) - 1):
                    if later[j][1] >= base - 1.5 and later[j + 1][1] >= base - 1.5:
                        d3_recovery = later[j][0] - dist_t0
                        break

    cons_main = stats.mainline_requested == stats.mainline_admitted + len(queue_main)
    cons_ramp = stats.ramp_requested == stats.ramp_merged + len(queue_ramp)
    cons_total = (stats.mainline_admitted + stats.ramp_merged
                  == stats.exits + len(vehicles))
    mae = lambda m, a, b: forecast_mae(fc_log, m, a, b, "D4")
    half = sc.warmup_s + window_s / 2.0

    n_adm = stats.mainline_admitted + stats.ramp_merged
    summary = {
        **{k: v for k, v in asdict(sc).items()
           if k not in ("save_trajectories", "traj_interval_s")},
        "overlap_error": int(overlap_error),
        "mainline_requested": stats.mainline_requested,
        "mainline_admitted": stats.mainline_admitted,
        "mainline_queue_end": len(queue_main),
        "ramp_requested": stats.ramp_requested,
        "ramp_merged": stats.ramp_merged,
        "ramp_queue_end": len(queue_ramp),
        "exits": stats.exits,
        "on_road_end": len(vehicles),
        "n_cv_admitted": len(cv_ids),
        "cv_fraction_admitted": len(cv_ids) / n_adm if n_adm else 0.0,
        "exit_rate_vph_window": exits_in_window / window_s * 3600.0,
        "d4_mean_flow_vph": det_stat("D4", "flow_veh_h", lambda v: sum(v) / len(v)),
        "d3_mean_speed_mps": det_stat("D3", "space_mean_speed_mps", lambda v: sum(v) / len(v)),
        "d3_min_speed_mps": det_stat("D3", "space_mean_speed_mps", min),
        "d2_mean_speed_mps": det_stat("D2", "space_mean_speed_mps", lambda v: sum(v) / len(v)),
        "total_delay_veh_h": delay_total_s / 3600.0,
        "mean_delay_s_per_request": delay_total_s / n_requested if n_requested else 0.0,
        "peak_ramp_queue": ramp_queue_peak,
        "mean_ramp_queue": ramp_queue_sum / n_steps,
        "peak_mainline_queue": main_queue_peak,
        "min_gap_m": min_gap if min_gap != float("inf") else None,
        "min_accel_mps2": min_accel,
        "hard_brake_vehicle_steps": hard_brake_steps,
        "entry_blocked_gap": stats.entry_blocked_gap,
        "entry_blocked_braking": stats.entry_blocked_braking,
        "ramp_blocked_front": stats.ramp_blocked_front,
        "ramp_blocked_rear": stats.ramp_blocked_rear,
        "ramp_blocked_braking": stats.ramp_blocked_braking,
        "mean_meter_rate_vph": (
            sum(c["value"] for c in ctrl_log if c["value"] is not None)
            / max(1, sum(1 for c in ctrl_log if c["value"] is not None))
            if any(c["value"] is not None for c in ctrl_log) else None),
        "acc_updates": acc.updates,
        "acc_fallback_updates": acc.fallback_updates,
        "acc_clipped_updates": acc.clipped_updates,
        "dist_vehicle_id": dist_id,
        "dist_n_tracked": len(profile_rows),
        "dist_max_dip_mps": max(dips) if dips else None,
        "dist_mean_dip_mps": sum(dips) / len(dips) if dips else None,
        "dist_n_affected": n_affected,
        "dist_max_rank_affected": max_rank_affected,
        "dist_n_recovered": n_recovered,
        "dist_n_unrecovered": n_unrecovered,
        "dist_recovery_fraction": recovery_fraction,
        "dist_start_position_m": disturbance_info["start_position_m"],
        "dist_start_speed_mps": disturbance_info["start_speed_mps"],
        "dist_braking_steps": disturbance_info["braking_steps"],
        "dist_actual_min_brake_mps2": disturbance_info["actual_min_brake_mps2"],
        "dist_mean_recovery_s": (sum(rec_after_min) / len(rec_after_min)
                                 if rec_after_min else None),
        "dist_duration_s": max(rec_abs) if rec_abs else None,
        "d3_speed_dip_mps": d3_dip,
        "d3_recovery_s": d3_recovery,
        "fc_mae_ma_dev": mae("moving_average", sc.warmup_s, half),
        "fc_mae_holt_dev": mae("holt_linear", sc.warmup_s, half),
        "fc_mae_ma_eval": mae("moving_average", half, sc.duration_s + 1),
        "fc_mae_holt_eval": mae("holt_linear", half, sc.duration_s + 1),
        "conserved_mainline": int(cons_main),
        "conserved_ramp": int(cons_ramp),
        "conserved_total": int(cons_total),
        "wall_time_s": time.time() - wall_start,
    }

    # Outputs
    write_csv(out_dir / "detectors.csv", det_rows)
    write_csv(out_dir / "observations.csv", obs_log)
    write_csv(out_dir / "estimates.csv", est_log)
    write_csv(out_dir / "forecasts.csv", fc_log)
    write_csv(out_dir / "controller_log.csv", ctrl_log)
    write_csv(out_dir / "queues.csv", queue_log)
    write_csv(out_dir / "disturbance_profile.csv", profile_rows)
    write_csv(out_dir / "disturbance_event.csv", [disturbance_info])
    if sc.save_trajectories:
        write_csv(out_dir / "trajectories.csv", traj_log)
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    if verbose:
        ok = "OK" if (cons_main and cons_ramp and cons_total and not overlap_error) else "CHECK"
        print(f"  {sc.scenario_id:<22} seed {s}  [{ok}]  "
              f"exits/h {summary['exit_rate_vph_window']:6.0f}  "
              f"ramp queue end {len(queue_ramp):3d}  "
              f"delay {summary['total_delay_veh_h']:6.2f} veh-h  "
              f"min gap {summary['min_gap_m'] if summary['min_gap_m'] is None else round(summary['min_gap_m'], 1)}  "
              f"({summary['wall_time_s']:.0f} s)")
    return summary


# ---------------------------------------------------------------------------
# Command line driver
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--penetrations", type=float, nargs="+", default=[0.0, 0.2, 0.5, 0.7])
    ap.add_argument("--seeds", type=int, nargs="+", default=[1])
    ap.add_argument("--ramp-control", nargs="+", default=["fixed"],
                    choices=["none", "fixed", "alinea"])
    ap.add_argument("--duration", type=float, default=1200.0)
    ap.add_argument("--warmup", type=float, default=300.0)
    ap.add_argument("--disturbance-time", type=float, default=600.0)
    ap.add_argument("--no-disturbance", action="store_true")
    ap.add_argument("--disturbance-mile", type=float, default=2.0,
                    help="where along the road (miles) the braking vehicle is chosen")
    ap.add_argument("--mainline-demand", type=float, default=1500.0)
    ap.add_argument("--ramp-demand", type=float, default=400.0)
    ap.add_argument("--sensor-dropout", type=float, default=0.0)
    ap.add_argument("--actuator-lag", type=float, default=0.3)
    ap.add_argument("--obs-missing", type=float, default=0.0)
    ap.add_argument("--obs-delay", type=int, default=0)
    ap.add_argument("--obs-noise", type=float, default=0.0)
    ap.add_argument("--ramp-entry", default="match_leader",
                    choices=["match_leader", "fixed"],
                    help="ramp vehicle entry speed (see Scenario.ramp_entry_mode)")
    ap.add_argument("--trajectories", action="store_true",
                    help="save vehicle trajectories (first seed only; large files)")
    ap.add_argument("--out", default="results")
    ap.add_argument("--quick", action="store_true",
                    help="short smoke test: 600 s, disturbance at 400 s")
    args = ap.parse_args()

    if args.quick:
        args.duration, args.warmup, args.disturbance_time = 600.0, 200.0, 400.0

    out = Path(args.out)
    out.mkdir(exist_ok=True)
    rows = []
    n_total = len(args.ramp_control) * len(args.penetrations) * len(args.seeds)
    print(f"Running {n_total} simulation(s)...")
    for ctrl in args.ramp_control:
        for pen in args.penetrations:
            for i, seed in enumerate(args.seeds):
                sc = Scenario(
                    seed=seed, duration_s=args.duration, warmup_s=args.warmup,
                    mainline_demand_vph=args.mainline_demand,
                    ramp_demand_vph=args.ramp_demand, cv_penetration=pen,
                    ramp_control=ctrl, ramp_entry_mode=args.ramp_entry,
                    disturbance=not args.no_disturbance,
                    disturbance_time_s=args.disturbance_time,
                    disturbance_target_m=args.disturbance_mile * MILE,
                    sensor_dropout_prob=args.sensor_dropout,
                    actuator_lag_s=args.actuator_lag,
                    obs_missing_prob=args.obs_missing,
                    obs_delay_intervals=args.obs_delay,
                    obs_count_noise_sd=args.obs_noise,
                    save_trajectories=args.trajectories and i == 0)
                rows.append(run_scenario(sc, out))

    # Append to (or create) the combined run table used by analysis.py
    summary_path = out / "runs_summary.csv"
    existing = []
    if summary_path.exists():
        with open(summary_path, newline="") as f:
            existing = list(csv.DictReader(f))
    new_keys = {(r["scenario_id"], str(r["seed"])) for r in rows}
    kept = [r for r in existing if (r["scenario_id"], r["seed"]) not in new_keys]
    all_rows = kept + rows
    fieldnames = list(rows[0])
    write_csv(summary_path, all_rows, fieldnames)
    print(f"\nWrote {summary_path} ({len(all_rows)} runs). Next: python analysis.py")


if __name__ == "__main__":
    main()
