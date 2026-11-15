
"""Compare matched braking/no-braking runs using detector time series.

Example:
  python compare_disturbance.py --with-braking results_with_braking \
      --without-braking results_without_braking --out disturbance_comparison

Positive speed deficit = speed in no-braking run minus speed in braking run.
Do not interpret as a perfectly isolated causal effect: vehicle trajectories
and populations can diverge even when demand seeds are paired.
"""
import argparse
import csv
from pathlib import Path


def read_csv(path):
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows, fields):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def number(v):
    if v is None or v == "":
        return None
    return float(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--with-braking", default="results_with_braking")
    ap.add_argument("--without-braking", default="results_without_braking")
    ap.add_argument("--out", default="disturbance_comparison")
    args = ap.parse_args()

    with_root = Path(args.with_braking)
    without_root = Path(args.without_braking)

    with_runs = read_csv(with_root / "runs_summary.csv")
    without_runs = read_csv(without_root / "runs_summary.csv")

    baselines = {
        (r["ramp_control"], r["cv_penetration"], r["seed"]): r
        for r in without_runs
        if r.get("disturbance") in ("False", "0", "0.0")
    }

    details, summaries = [], []

    for run in with_runs:
        if run.get("disturbance") not in ("True", "1", "1.0"):
            continue

        key = (
            run["ramp_control"],
            run["cv_penetration"],
            run["seed"]
        )

        baseline = baselines.get(key)

        if baseline is None:
            print("Skipping unmatched scenario:", key)
            continue

        wpath = (
            with_root
            / run["scenario_id"]
            / f"seed_{run['seed']}"
            / "detectors.csv"
        )

        bpath = (
            without_root
            / baseline["scenario_id"]
            / f"seed_{baseline['seed']}"
            / "detectors.csv"
        )

        if not wpath.exists() or not bpath.exists():
            print("Skipping missing detector CSV:", key)
            continue

        bdet = {
            (r["sensor_id"], r["time_s"]): r
            for r in read_csv(bpath)
        }

        start = number(run.get("disturbance_time_s"))

        if start is None:
            continue

        deficits = []

        for r in read_csv(wpath):
            t = number(r["time_s"])

            if t is None or t <= start:
                continue

            base = bdet.get((r["sensor_id"], r["time_s"]))

            if base is None:
                continue

            # Space-mean speed is an interval estimate
            # based on crossing vehicles.
            bw = number(r.get("space_mean_speed_mps"))
            bn = number(base.get("space_mean_speed_mps"))

            speed_deficit = (
                None
                if bw is None or bn is None
                else bn - bw
            )

            flow_difference = (
                number(base["flow_veh_h"])
                - number(r["flow_veh_h"])
            )

            details.append({
                "ramp_control": key[0],
                "cv_penetration": key[1],
                "seed": key[2],
                "sensor_id": r["sensor_id"],
                "time_s": t,
                "seconds_after_event": round(t - start, 3),
                "speed_no_braking_mps": bn,
                "speed_braking_mps": bw,
                "speed_deficit_mps": speed_deficit,
                "flow_deficit_vph": flow_difference
            })

            if speed_deficit is not None:
                deficits.append(speed_deficit)

        summaries.append({
            "ramp_control": key[0],
            "cv_penetration": key[1],
            "seed": key[2],
            "n_speed_comparisons": len(deficits),
            "mean_speed_deficit_mps": (
                sum(deficits) / len(deficits)
                if deficits else None
            ),
            "max_speed_deficit_mps": (
                max(deficits) if deficits else None
            ),
            "mean_delay_change_s": (
                number(run.get("mean_delay_s_per_request"))
                - number(baseline.get("mean_delay_s_per_request"))
            )
        })

    if not summaries:
        raise SystemExit(
            "No matched braking/no-braking scenarios found. "
            "Check paths and seeds."
        )

    out = Path(args.out)

    write_csv(
        out / "paired_detector_intervals.csv",
        details,
        [
            "ramp_control",
            "cv_penetration",
            "seed",
            "sensor_id",
            "time_s",
            "seconds_after_event",
            "speed_no_braking_mps",
            "speed_braking_mps",
            "speed_deficit_mps",
            "flow_deficit_vph"
        ]
    )

    write_csv(
        out / "paired_run_summary.csv",
        summaries,
        [
            "ramp_control",
            "cv_penetration",
            "seed",
            "n_speed_comparisons",
            "mean_speed_deficit_mps",
            "max_speed_deficit_mps",
            "mean_delay_change_s"
        ]
    )

    print(
        f"Matched {len(summaries)} runs; "
        f"saved comparison CSVs to {out}"
    )

    print(
        "NOTE: 30-second detector data cannot fully "
        "resolve a 3-second braking pulse."
    )


if __name__ == "__main__":
    main()
