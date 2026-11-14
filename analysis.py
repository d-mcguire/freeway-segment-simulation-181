"""Analysis of simulation results: tables across seeds and figures.

Reads  results/runs_summary.csv  and the per-run folders written by
run_simulation.py, and writes tables and figures to results/figures/.

Usage:  python analysis.py [--results results]

Dependencies: matplotlib (and its numpy). No pandas.

Aggregation across seeds
  * mean, standard deviation, min, max and a 95 % confidence interval (t-based)
    for every metric, per scenario.
  * PAIRED differences versus the 0 % CV run with the same seed (same arrival
    sequence), because the seed fixes arrivals and CV tags independently of the
    scenario. The sign of the per-seed difference is reported.
"""
import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MILE = 1609.344
T_CRIT_95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
             6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262}

TEXT_FIELDS = {"scenario_id", "ramp_control", "ramp_entry_mode"}

TABLE_METRICS = [
    ("exit_rate_vph_window", "Exit rate after warm-up (veh/h)"),
    ("d4_mean_flow_vph", "Flow at D4, mile 2.8 (veh/h)"),
    ("total_delay_veh_h", "Total delay (veh-h)"),
    ("mean_delay_s_per_request", "Mean delay per requested vehicle (s)"),
    ("ramp_queue_end", "Ramp queue at end (veh, unmet demand)"),
    ("peak_ramp_queue", "Peak ramp queue (veh)"),
    ("mainline_queue_end", "Upstream mainline queue at end (veh)"),
    ("d3_min_speed_mps", "Lowest 30 s speed at D3 (m/s)"),
    ("min_gap_m", "Minimum gap (m)"),
    ("min_accel_mps2", "Hardest braking (m/s^2)"),
    ("hard_brake_vehicle_steps", "Vehicle-steps below -3 m/s^2"),
    ("dist_n_affected", "Followers affected by disturbance (of 60)"),
    ("dist_max_rank_affected", "Furthest follower affected (rank)"),
    ("dist_max_dip_mps", "Largest follower speed dip (m/s)"),
    ("dist_mean_dip_mps", "Mean follower speed dip (m/s)"),
    ("d3_speed_dip_mps", "D3 speed dip (m/s)"),
    ("d3_recovery_s", "D3 recovery time (s; blank = not recovered)"),
    ("dist_mean_recovery_s", "Mean recovery after own min speed (s; censored)"),
    ("fc_mae_ma_eval", "Forecast MAE moving average, eval (veh/h)"),
    ("fc_mae_holt_eval", "Forecast MAE Holt linear, eval (veh/h)"),
]


# ---------------------------------------------------------------------------
# Loading and aggregation
# ---------------------------------------------------------------------------
def to_number(value):
    if value in ("", None):
        return None
    if value in ("True", "False"):
        return 1.0 if value == "True" else 0.0
    try:
        return float(value)
    except ValueError:
        return value


def load_runs(results_dir):
    path = results_dir / "runs_summary.csv"
    if not path.exists():
        raise SystemExit(f"{path} not found. Run run_simulation.py first.")
    rows = []
    with open(path, newline="") as f:
        for raw in csv.DictReader(f):
            rows.append({k: (v if k in TEXT_FIELDS else to_number(v))
                         for k, v in raw.items()})
    return rows


def read_csv_rows(path):
    if not path.exists():
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def run_dir(results_dir, row):
    return results_dir / row["scenario_id"] / f"seed_{int(row['seed'])}"


def summarize(values):
    vals = [v for v in values if isinstance(v, (int, float))]
    n = len(vals)
    if n == 0:
        return {"n": 0, "mean": None, "sd": None, "min": None, "max": None, "ci95": None}
    mean = statistics.mean(vals)
    sd = statistics.stdev(vals) if n > 1 else None
    ci = (T_CRIT_95.get(n - 1, 1.96) * sd / n ** 0.5) if sd is not None else None
    return {"n": n, "mean": mean, "sd": sd, "min": min(vals), "max": max(vals), "ci95": ci}


def fmt(x, digits=2):
    return "" if x is None else f"{x:.{digits}f}"


def settings_key(r):
    """Everything except CV penetration, so runs can be paired with their 0 % baseline."""
    return (r["ramp_control"], r["seed"], r["sensor_dropout_prob"],
            r["obs_missing_prob"], r["obs_delay_intervals"],
            r["obs_count_noise_sd"], r["disturbance"],
            r["mainline_demand_vph"], r["ramp_demand_vph"], r["actuator_lag_s"])


def aggregate_table(rows, out_path):
    groups = defaultdict(list)
    for r in rows:
        groups[r["scenario_id"]].append(r)
    out = []
    for sid in sorted(groups):
        rs_ = groups[sid]
        for key, label in TABLE_METRICS:
            s = summarize([r.get(key) for r in rs_])
            out.append({"scenario_id": sid, "metric": key, "label": label,
                        "n_seeds": s["n"], "mean": fmt(s["mean"], 3),
                        "sd": fmt(s["sd"], 3), "min": fmt(s["min"], 3),
                        "max": fmt(s["max"], 3), "ci95_halfwidth": fmt(s["ci95"], 3)})
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0]))
        w.writeheader()
        w.writerows(out)
    return groups


def paired_table(rows, out_path):
    base = {settings_key(r): r for r in rows if r["cv_penetration"] == 0.0}
    groups = defaultdict(list)
    for r in rows:
        if r["cv_penetration"] > 0 and settings_key(r) in base:
            groups[r["scenario_id"]].append((r, base[settings_key(r)]))
    out = []
    for sid in sorted(groups):
        pairs = groups[sid]
        for key, label in TABLE_METRICS:
            diffs = [a[key] - b[key] for a, b in pairs
                     if isinstance(a.get(key), (int, float))
                     and isinstance(b.get(key), (int, float))]
            if not diffs:
                continue
            s = summarize(diffs)
            out.append({"scenario_id": sid, "metric": key, "n_pairs": len(diffs),
                        "mean_diff_vs_0pct": fmt(s["mean"], 3),
                        "sd_of_diff": fmt(s["sd"], 3),
                        "seeds_lower": sum(d < 0 for d in diffs),
                        "seeds_higher": sum(d > 0 for d in diffs)})
    if out:
        with open(out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(out[0]))
            w.writeheader()
            w.writerows(out)
    return out


def print_headline(groups, paired):
    print("\nHeadline metrics (mean +/- sd over seeds)")
    keys = ["exit_rate_vph_window", "total_delay_veh_h", "ramp_queue_end",
            "dist_n_affected", "dist_max_dip_mps", "min_accel_mps2"]
    print(f"{'scenario':<22}{'seeds':>5}  " + "  ".join(f"{k[:19]:>19}" for k in keys))
    for sid in sorted(groups):
        cells = []
        for k in keys:
            s = summarize([r.get(k) for r in groups[sid]])
            cells.append(f"{fmt(s['mean'], 1)}+/-{fmt(s['sd'], 1)}".rjust(19)
                         if s["mean"] is not None else " " * 19)
        print(f"{sid:<22}{len(groups[sid]):>5}  " + "  ".join(cells))
    if paired:
        print("\nPaired difference vs 0 % CV (same seed): mean, and how many seeds went lower/higher")
        for row in paired:
            if row["metric"] in ("total_delay_veh_h", "dist_n_affected", "exit_rate_vph_window"):
                print(f"  {row['scenario_id']:<22}{row['metric']:<24}"
                      f"{row['mean_diff_vs_0pct']:>9}   lower {row['seeds_lower']} / higher {row['seeds_higher']} of {row['n_pairs']}")


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
def plot_metrics_vs_penetration(rows, fig_dir):
    metrics = [("exit_rate_vph_window", "Exit rate after warm-up (veh/h)"),
               ("total_delay_veh_h", "Total delay (veh-h)"),
               ("ramp_queue_end", "Ramp queue at end (veh)"),
               ("dist_n_affected", "Followers affected by disturbance"),
               ("dist_max_dip_mps", "Largest follower speed dip (m/s)"),
               ("min_accel_mps2", "Hardest braking (m/s$^2$)")]
    controls = sorted({r["ramp_control"] for r in rows})
    fig, axes = plt.subplots(2, 3, figsize=(13, 7))
    for ax, (key, label) in zip(axes.ravel(), metrics):
        for i, ctrl in enumerate(controls):
            color = f"C{i}"
            by_pen = defaultdict(list)
            for r in rows:
                if r["ramp_control"] == ctrl and isinstance(r.get(key), (int, float)):
                    by_pen[r["cv_penetration"]].append(r[key])
            pens = sorted(by_pen)
            for p in pens:                       # individual seeds
                ax.scatter([100 * p] * len(by_pen[p]), by_pen[p], s=14, alpha=0.35, color=color)
            means = [statistics.mean(by_pen[p]) for p in pens]
            sds = [statistics.stdev(by_pen[p]) if len(by_pen[p]) > 1 else 0 for p in pens]
            ax.errorbar([100 * p for p in pens], means, yerr=sds, color=color,
                        marker="o", capsize=3, label=f"{ctrl} meter")
        ax.set_xlabel("CV penetration (%)")
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Metrics vs CV penetration (dots = seeds; line = mean $\\pm$ 1 sd)")
    fig.tight_layout()
    fig.savefig(fig_dir / "metrics_vs_penetration.png", dpi=150)
    plt.close(fig)


def plot_detectors(results_dir, row, fig_dir):
    data = read_csv_rows(run_dir(results_dir, row) / "detectors.csv")
    if not data:
        return
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    by_sensor = defaultdict(list)
    for d in data:
        by_sensor[d["name"]].append(d)
    for name, ds in by_sensor.items():
        t = [float(d["time_s"]) for d in ds]
        axes[0].plot(t, [float(d["flow_veh_h"]) for d in ds], label=name, lw=1.2)
        speeds = [float(d["space_mean_speed_mps"]) if d["space_mean_speed_mps"] else float("nan") for d in ds]
        axes[1].plot(t, speeds, label=name, lw=1.2)
    for ax in axes:
        if row.get("disturbance"):
            ax.axvspan(row["disturbance_time_s"], row["disturbance_time_s"] + row["brake_duration_s"],
                       color="gray", alpha=0.4)
        ax.axvline(row["warmup_s"], color="k", ls=":", lw=0.8)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("Flow (veh/h), 30 s intervals")
    axes[1].set_ylabel("Space-mean speed (m/s)")
    axes[1].set_xlabel("Time (s)  [gray = disturbance, dotted = end of warm-up]")
    axes[0].legend(fontsize=7, ncol=2)
    axes[0].set_title(f"Virtual detectors: {row['scenario_id']}, seed {int(row['seed'])}")
    fig.tight_layout()
    fig.savefig(fig_dir / f"detectors_{row['scenario_id']}_seed{int(row['seed'])}.png", dpi=140)
    plt.close(fig)


def plot_time_space(results_dir, row, fig_dir):
    data = read_csv_rows(run_dir(results_dir, row) / "trajectories.csv")
    if not data:
        return
    t = [float(d["time_s"]) for d in data]
    x = [float(d["position_m"]) / MILE for d in data]
    v = [float(d["speed_mps"]) for d in data]
    fig, ax = plt.subplots(figsize=(10, 6))
    sc = ax.scatter(t, x, c=v, s=0.6, cmap="viridis", vmin=0, vmax=30, rasterized=True)
    ax.axhline(2.5, color="r", ls="--", lw=0.8, label="ramp merge (2.5 mi)")
    if row.get("disturbance"):
        ax.axvline(row["disturbance_time_s"], color="k", ls=":", lw=1, label="disturbance")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (miles)")
    ax.set_title(f"Time-space diagram: {row['scenario_id']}, seed {int(row['seed'])}")
    ax.legend(loc="upper left", fontsize=8)
    fig.colorbar(sc, label="Speed (m/s)")
    fig.tight_layout()
    fig.savefig(fig_dir / f"time_space_{row['scenario_id']}_seed{int(row['seed'])}.png", dpi=150)
    plt.close(fig)


def plot_disturbance_profile(results_dir, rows, fig_dir):
    by_ctrl = defaultdict(lambda: defaultdict(lambda: defaultdict(list)))
    for r in rows:
        for d in read_csv_rows(run_dir(results_dir, r) / "disturbance_profile.csv"):
            by_ctrl[r["ramp_control"]][r["cv_penetration"]][int(d["rank"])].append(float(d["dip_mps"]))
    for ctrl, pens in by_ctrl.items():
        fig, ax = plt.subplots(figsize=(9, 5))
        for i, p in enumerate(sorted(pens)):
            ranks = sorted(pens[p])
            ax.plot(ranks, [statistics.mean(pens[p][k]) for k in ranks],
                    label=f"{round(100 * p)} % CV", color=f"C{i}", lw=1.5)
        ax.axhline(1.0, color="k", ls=":", lw=0.8)
        ax.set_xlabel("Follower rank behind the braking vehicle (1 = directly behind)")
        ax.set_ylabel("Mean speed dip (m/s), averaged over seeds")
        ax.set_title(f"Disturbance propagation ({ctrl} meter); dotted line = 'affected' threshold")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(fig_dir / f"disturbance_profile_{ctrl}.png", dpi=150)
        plt.close(fig)


def plot_queues(results_dir, rows, fig_dir):
    controls = sorted({r["ramp_control"] for r in rows})
    fig, axes = plt.subplots(1, len(controls), figsize=(5.5 * len(controls), 4.5),
                             sharey=True, squeeze=False)
    for ax, ctrl in zip(axes[0], controls):
        series = defaultdict(lambda: defaultdict(list))
        for r in rows:
            if r["ramp_control"] != ctrl:
                continue
            for q in read_csv_rows(run_dir(results_dir, r) / "queues.csv"):
                series[r["cv_penetration"]][float(q["time_s"])].append(float(q["ramp_queue"]))
        for i, p in enumerate(sorted(series)):
            ts = sorted(series[p])
            ax.plot(ts, [statistics.mean(series[p][t]) for t in ts],
                    label=f"{round(100 * p)} % CV", color=f"C{i}", lw=1.3)
        ax.set_title(f"Ramp queue, {ctrl} meter (mean over seeds)")
        ax.set_xlabel("Time (s)")
        ax.grid(alpha=0.3)
    axes[0][0].set_ylabel("Vehicles waiting on ramp")
    axes[0][0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(fig_dir / "ramp_queues.png", dpi=150)
    plt.close(fig)


def plot_forecasts(groups, fig_dir):
    sids = sorted(groups)
    ma = [summarize([r.get("fc_mae_ma_eval") for r in groups[s]])["mean"] for s in sids]
    holt = [summarize([r.get("fc_mae_holt_eval") for r in groups[s]])["mean"] for s in sids]
    if all(m is None for m in ma):
        return
    xs = range(len(sids))
    fig, ax = plt.subplots(figsize=(max(7, 1.4 * len(sids)), 4.5))
    ax.bar([x - 0.2 for x in xs], [m or 0 for m in ma], width=0.4, label="moving average (baseline)")
    ax.bar([x + 0.2 for x in xs], [h or 0 for h in holt], width=0.4, label="Holt linear")
    ax.set_xticks(list(xs))
    ax.set_xticklabels(sids, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("Flow forecast MAE at D4, 60 s ahead (veh/h)")
    ax.set_title("Forecast error on the evaluation period (mean over seeds)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "forecast_mae.png", dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results")
    args = ap.parse_args()
    results_dir = Path(args.results)
    fig_dir = results_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)

    rows = load_runs(results_dir)
    bad = [r for r in rows if not (r["conserved_mainline"] and r["conserved_ramp"]
                                   and r["conserved_total"]) or r["overlap_error"]]
    print(f"Loaded {len(rows)} runs ({len({r['seed'] for r in rows})} seeds, "
          f"{len({r['scenario_id'] for r in rows})} scenarios).")
    if bad:
        print(f"WARNING: {len(bad)} run(s) failed a conservation or overlap check:",
              [(r["scenario_id"], int(r["seed"])) for r in bad])

    groups = aggregate_table(rows, results_dir / "analysis_summary.csv")
    paired = paired_table(rows, results_dir / "paired_differences.csv")
    print_headline(groups, paired)

    plot_metrics_vs_penetration(rows, fig_dir)
    plot_disturbance_profile(results_dir, rows, fig_dir)
    plot_queues(results_dir, rows, fig_dir)
    plot_forecasts(groups, fig_dir)
    first_seed = min(r["seed"] for r in rows)
    for r in rows:
        if r["seed"] == first_seed:
            plot_detectors(results_dir, r, fig_dir)
            plot_time_space(results_dir, r, fig_dir)

    print(f"\nTables: {results_dir/'analysis_summary.csv'}, {results_dir/'paired_differences.csv'}")
    print(f"Figures: {fig_dir}")


if __name__ == "__main__":
    main()
