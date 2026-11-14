"""Controllers.

Part A - Ramp metering (shared TMC foundation)
    NoMeter          : ramp vehicles are released whenever the merge gap test passes
    FixedRateMeter   : constant release rate (the existing 600 veh/h baseline)
    AlineaMeter      : integral feedback on downstream density (ALINEA-style)

Part B - ACC controller for connected vehicles (CAV pathway)
    ACCController    : IDM-based adaptive cruise control with a stated control law,
                       measured inputs, update interval, acceleration bounds,
                       command-to-motion lag, and a fallback to human-like driving.

Neither controller receives simulator truth beyond what a real sensor would
provide: the meter sees the estimated state (from observations.py) and the ramp
queue length; the ACC sees its own speed, the gap to its leader and the
leader's speed.
"""
import math
from dataclasses import dataclass

from idm import IDMParams, idm_accel, equilibrium_flow_veh_per_h

# ---------------------------------------------------------------------------
# Shared parameter sets (same values as platoon_experiment.py)
# ---------------------------------------------------------------------------
HDV_PARAMS = IDMParams()
CV_PARAMS = IDMParams(
    v0=30.0,
    T=0.7 * HDV_PARAMS.T,   # lambda_T = 0.7  -> 1.05 s
    s0=2.0,
    a=1.5 * HDV_PARAMS.a,   # lambda_a = 1.5  -> 1.5 m/s^2
    b=1.5,
    delta=4.0,
    length=5.0,
)


def critical_density_veh_km(params=HDV_PARAMS):
    """Density (veh/km/lane) at the IDM equilibrium flow maximum."""
    best_v, best_q = 0.0, 0.0
    v = 0.5
    while v < params.v0 - 0.05:
        q = equilibrium_flow_veh_per_h(v, params)
        if q > best_q:
            best_v, best_q = v, q
        v += 0.05
    return best_q / (best_v * 3.6), best_q, best_v


# ---------------------------------------------------------------------------
# Part A: ramp metering
# ---------------------------------------------------------------------------
def make_command(time_s, value, lower, upper, fallback, reason, **inputs):
    """Control-command record (Design Project Guide format) plus a decision trace."""
    cmd = {
        "issue_time_s": time_s,
        "apply_time_s": time_s,
        "actuator_id": "ramp_meter",
        "action": "set_release_rate",
        "value": value,
        "units": "veh/h",
        "lower_bound": lower,
        "upper_bound": upper,
        "fallback": fallback,
        "reason": reason,
    }
    cmd.update(inputs)
    return cmd


class NoMeter:
    name = "none"

    def decide(self, time_s, estimate, ramp_queue):
        return make_command(time_s, None, None, None, None, "unmetered",
                            est_density_veh_km=None, ramp_queue=ramp_queue)


class FixedRateMeter:
    name = "fixed"

    def __init__(self, rate_vph=600.0):
        self.rate = rate_vph

    def decide(self, time_s, estimate, ramp_queue):
        return make_command(time_s, self.rate, self.rate, self.rate, self.rate,
                            "fixed_rate", est_density_veh_km=None,
                            ramp_queue=ramp_queue)


class AlineaMeter:
    """r(k) = r(k-1) + K_R * (density_target - density_measured), clipped.

    The measured density comes from the state estimator at `sensor_id`
    (downstream of the merge). Fallbacks:
      * estimate missing or older than max_age_intervals -> fallback_rate
      * ramp queue at/over storage_veh                    -> release at max_rate

    K_R, the target fraction and the bounds are PLACEHOLDERS to be tuned and
    justified against the course material (TFD Sec. 21.3) before they are reported.
    """
    name = "alinea"

    def __init__(self, sensor_id="D4", gain_vph_per_veh_km=50.0,
                 target_fraction=0.95, min_rate_vph=200.0, max_rate_vph=900.0,
                 initial_rate_vph=600.0, fallback_rate_vph=600.0,
                 max_age_intervals=3, storage_veh=30):
        self.sensor_id = sensor_id
        self.gain = gain_vph_per_veh_km
        self.k_crit, self.q_crit, self.v_crit = critical_density_veh_km()
        self.k_target = target_fraction * self.k_crit
        self.r_min, self.r_max = min_rate_vph, max_rate_vph
        self.rate = initial_rate_vph
        self.fallback = fallback_rate_vph
        self.max_age = max_age_intervals
        self.storage = storage_veh

    def decide(self, time_s, estimate, ramp_queue):
        est = estimate.get(self.sensor_id) if estimate else None
        usable = (est is not None and est["density_veh_km"] is not None
                  and est["age_intervals"] <= self.max_age)
        k_meas = est["density_veh_km"] if usable else None

        if not usable:
            self.rate = self.fallback
            reason = "fallback_no_data"
        else:
            self.rate += self.gain * (self.k_target - k_meas)
            self.rate = min(self.r_max, max(self.r_min, self.rate))
            reason = "feedback"

        if ramp_queue >= self.storage:
            self.rate = self.r_max
            reason = "queue_override"

        return make_command(time_s, self.rate, self.r_min, self.r_max,
                            self.fallback, reason, est_density_veh_km=k_meas,
                            ramp_queue=ramp_queue)


def make_ramp_controller(kind, fixed_rate_vph=600.0, meter_sensor="D4", **kwargs):
    if kind == "none":
        return NoMeter()
    if kind == "fixed":
        return FixedRateMeter(fixed_rate_vph)
    if kind == "alinea":
        return AlineaMeter(sensor_id=meter_sensor, **kwargs)
    raise ValueError(f"Unknown ramp controller '{kind}'")


# ---------------------------------------------------------------------------
# Part B: ACC controller for connected vehicles
# ---------------------------------------------------------------------------
@dataclass
class ACCConfig:
    cv_params: IDMParams = CV_PARAMS
    fallback_params: IDMParams = HDV_PARAMS
    a_min_mps2: float = -3.5          # PLACEHOLDER braking bound - cite a source
    a_max_mps2: float = None          # defaults to cv_params.a
    update_interval_s: float = 0.1    # controller update period
    actuator_lag_s: float = 0.3       # PLACEHOLDER first-order command-to-motion lag
    sensor_dropout_prob: float = 0.0  # per-update chance the radar input is lost


class ACCController:
    """IDM-based ACC.

    Control law   : a_cmd = IDM(v, gap, dv; CV parameters)
    Inputs        : own speed v, bumper-to-bumper gap, closing speed dv = v - v_leader
    Bounds        : a_min <= a_cmd <= a_max
    Update        : every update_interval_s, command held in between
    Motion        : first-order lag, a_actual += (1 - exp(-dt/tau)) * (a_cmd - a_actual)
    Fallback      : on sensor dropout the vehicle reverts to the human (HDV)
                    parameter set with no bounds and no lag, for that update
    """

    def __init__(self, config=None):
        self.cfg = config or ACCConfig()
        self.a_max = (self.cfg.a_max_mps2 if self.cfg.a_max_mps2 is not None
                      else self.cfg.cv_params.a)
        self.state = {}
        self.updates = 0
        self.fallback_updates = 0
        self.clipped_updates = 0

    def compute(self, vehicle, leader, time_s, dt, rng):
        cfg = self.cfg
        st = self.state.get(vehicle.vehicle_id)
        if st is None:
            st = {"a_cmd": 0.0, "a_act": 0.0, "next_update": time_s,
                  "fallback": False}
            self.state[vehicle.vehicle_id] = st

        if time_s >= st["next_update"] - 1e-9:
            self.updates += 1
            st["next_update"] = time_s + cfg.update_interval_s
            dropout = (cfg.sensor_dropout_prob > 0
                       and rng.random() < cfg.sensor_dropout_prob)
            if leader is None:
                gap, dv = None, 0.0
            else:
                gap = leader.position_m - leader.params.length - vehicle.position_m
                dv = vehicle.speed_mps - leader.speed_mps

            if dropout:
                st["fallback"] = True
                self.fallback_updates += 1
                st["a_cmd"] = idm_accel(vehicle.speed_mps, gap, dv,
                                        cfg.fallback_params)
            else:
                st["fallback"] = False
                raw = idm_accel(vehicle.speed_mps, gap, dv, cfg.cv_params)
                st["a_cmd"] = min(self.a_max, max(cfg.a_min_mps2, raw))
                if st["a_cmd"] != raw:
                    self.clipped_updates += 1

        if st["fallback"] or cfg.actuator_lag_s <= 0:
            st["a_act"] = st["a_cmd"]
        else:
            alpha = 1.0 - math.exp(-dt / cfg.actuator_lag_s)
            st["a_act"] += alpha * (st["a_cmd"] - st["a_act"])
        return st["a_act"]

    def forget(self, vehicle_id):
        self.state.pop(vehicle_id, None)
