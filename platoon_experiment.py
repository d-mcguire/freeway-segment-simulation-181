
"""IDM braking-disturbance platoon experiment

Compare 0%, 20%, 50%, and 70% equipped followers
"""

import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from idm import (
    IDMParams,
    Vehicle,
    equilibrium_gap,
    follower_accel,
    step_vehicle,
)

# 1. Simulation Settings

DT = 0.1
DURATION_S = 180.0
N_VEHICLES = 41

INITIAL_SPEED_MPS = 20.0

BRAKE_START_S = 10.0
BRAKE_DURATION_S = 3.0
BRAKE_ACCEL_MPS2 = -2.0

LEADER_RECOVERY_MPS2 = 1.0

PENETRATIONS = (0.0, 0.20, 0.50, 0.70)

SEED = 181
OUTPUT = Path("platoon_results")

# 2. Vehicle Parameters

HDV = IDMParams(
    v0=30.0,
    T=1.5,
    s0=2.0,
    a=1.0,
    b=1.5,
    delta=4.0,
    length=5.0,
)

CV = IDMParams(
    v0=30.0,
    T=0.7 * HDV.T,
    s0=2.0,
    a=1.5 * HDV.a,
    b=1.5,
    delta=4.0,
    length=5.0,
)

# 3. Assign Equipped Vehicles

def equipment_assignment(penetration):
    """Assign equipped followers using a fixed seed."""

    rng = random.Random(SEED)

    follower_ids = list(range(1, N_VEHICLES))
    rng.shuffle(follower_ids)

    n_equipped = round(
        penetration * len(follower_ids)
    )

    return set(follower_ids[:n_equipped])


# 4. Create the Vehicle Platoon

def make_platoon(equipped_ids):
    """Initialize vehicles at their own IDM equilibrium gaps."""

    vehicles = [
        Vehicle(
            vehicle_id=0,
            position_m=0.0,
            speed_mps=INITIAL_SPEED_MPS,
            params=HDV,
        )
    ]

    for i in range(1, N_VEHICLES):

        params = CV if i in equipped_ids else HDV

        leader = vehicles[-1]

        gap = equilibrium_gap(
            INITIAL_SPEED_MPS,
            params
        )

        position = (
            leader.position_m
            - leader.params.length
            - gap
        )

        vehicles.append(
            Vehicle(
                vehicle_id=i,
                position_m=position,
                speed_mps=INITIAL_SPEED_MPS,
                params=params,
            )
        )

    return vehicles


# 5. Lead Vehicle Braking

def leader_acceleration(t, speed, dt):

    brake_end = (
        BRAKE_START_S + BRAKE_DURATION_S
    )

    if BRAKE_START_S <= t < brake_end:
        return BRAKE_ACCEL_MPS2

    if t >= brake_end and speed < INITIAL_SPEED_MPS:
        return min(
            LEADER_RECOVERY_MPS2,
            (INITIAL_SPEED_MPS - speed) / dt
        )

    return 0.0

# 6. Run One Penetration Scenario

def run_case(penetration, dt=DT):

    equipped_ids = equipment_assignment(
        penetration
    )

    vehicles = make_platoon(equipped_ids)

    records = []
    min_gap = float("inf")

    n_steps = round(DURATION_S / dt)

    for step in range(n_steps + 1):

        t = step * dt

        # Record vehicle states
        for car in vehicles:

            records.append({
                "penetration": penetration,
                "time_s": round(t, 8),
                "vehicle_id": car.vehicle_id,
                "equipped": int(
                    car.vehicle_id in equipped_ids
                ),
                "position_m": car.position_m,
                "speed_mps": car.speed_mps,
                "acceleration_mps2": car.accel_mps2,
            })

        if step == n_steps:
            break

        # Prescribed lead-vehicle acceleration
        accelerations = [
            leader_acceleration(
                t,
                vehicles[0].speed_mps,
                dt
            )
        ]

        # IDM follower accelerations
        for i in range(1, len(vehicles)):

            a = follower_accel(
                vehicles[i],
                vehicles[i - 1]
            )

            accelerations.append(a)

        # Update all vehicles
        for car, a in zip(
            vehicles, accelerations
        ):
            step_vehicle(car, a, dt)

        # Check bumper-to-bumper gaps
        for leader, follower in zip(
            vehicles, vehicles[1:]
        ):

            gap = (
                leader.position_m
                - leader.params.length
                - follower.position_m
            )

            min_gap = min(min_gap, gap)

            if gap <= 0:
                raise RuntimeError(
                    f"Vehicle overlap at t={t+dt:.2f}s, "
                    f"penetration={penetration:.0%}, "
                    f"follower={follower.vehicle_id}"
                )

    return records, min_gap, equipped_ids


# 7. Calculate Recovery Metrics

def summarize(records, penetration, min_gap, equipped_ids):

    by_id = {
        i: [] for i in range(N_VEHICLES)
    }

    for row in records:
        by_id[row["vehicle_id"]].append(row)

    brake_end = (
        BRAKE_START_S + BRAKE_DURATION_S
    )

    recovery_threshold = (
        0.95 * INITIAL_SPEED_MPS
    )

    recovery_times = []
    minimum_speeds = []

    for i in range(1, N_VEHICLES):

        rows = by_id[i]

        minimum_speeds.append(
            min(
                r["speed_mps"]
                for r in rows
                if r["time_s"] >= BRAKE_START_S
            )
        )

        recovery = None

        for j, row in enumerate(rows):

            t = row["time_s"]

            if t < brake_end:
                continue

            if row["speed_mps"] < recovery_threshold:
                continue

            # Require speed to stay recovered
            # for five continuous seconds
            window = [
                r for r in rows[j:]
                if r["time_s"] <= t + 5.0 + 1e-8
            ]

            if (
                window
                and window[-1]["time_s"] >= t + 5.0 - 1e-8
                and all(
                    r["speed_mps"] >= recovery_threshold
                    for r in window
                )
            ):
                recovery = t - brake_end
                break

        recovery_times.append(recovery)

    recovered = [
        t for t in recovery_times
        if t is not None
    ]

    return {
        "penetration_requested": penetration,
        "penetration_actual_followers":
            len(equipped_ids) / (N_VEHICLES - 1),
        "n_equipped_followers": len(equipped_ids),
        "min_gap_m": min_gap,
        "lowest_follower_speed_mps":
            min(minimum_speeds),
        "mean_recovery_after_brake_s":
            sum(recovered) / len(recovered)
            if recovered else None,
        "n_recovered": len(recovered),
        "n_followers": N_VEHICLES - 1,
    }


# 8. Save CSV Results

def save_csv(path, rows):

    with path.open("w", newline="") as file:

        writer = csv.DictWriter(
            file,
            fieldnames=list(rows[0])
        )

        writer.writeheader()
        writer.writerows(rows)


# 9. Plot Vehicle Speeds

def plot_speeds(records, penetration):

    plt.figure(figsize=(10, 5.5))

    selected_vehicles = [0, 1, 5, 10, 20, 30, 40]

    for i in selected_vehicles:

        rows = [
            r for r in records
            if r["vehicle_id"] == i
        ]

        times = [
            r["time_s"] for r in rows
        ]

        speeds = [
            r["speed_mps"] for r in rows
        ]

        plt.plot(
            times,
            speeds,
            label=f"Vehicle {i}",
            linewidth=1.4
        )

    plt.axvspan(
        BRAKE_START_S,
        BRAKE_START_S + BRAKE_DURATION_S,
        alpha=0.15,
        color="gray",
        label="Leader braking"
    )

    plt.xlabel("Time (s)")
    plt.ylabel("Speed (m/s)")

    plt.title(
        f"IDM Braking Disturbance: "
        f"{penetration:.0%} CV Penetration"
    )

    plt.grid(alpha=0.3)
    plt.legend(fontsize=7, ncol=3)
    plt.tight_layout()

    filename = (
        f"speeds_{round(100*penetration):02d}pct.png"
    )

    plt.savefig(
        OUTPUT / filename,
        dpi=170
    )

    plt.close()


# 10. Run All Four Scenarios

def main():

    OUTPUT.mkdir(exist_ok=True)

    summaries = []

    for penetration in PENETRATIONS:

        records, min_gap, equipped = run_case(
            penetration
        )

        summary = summarize(
            records,
            penetration,
            min_gap,
            equipped
        )

        summaries.append(summary)

        filename = (
            f"trajectories_"
            f"{round(100*penetration):02d}pct.csv"
        )

        save_csv(
            OUTPUT / filename,
            records
        )

        plot_speeds(
            records,
            penetration
        )

        print(
            f"{penetration:.0%} CV: "
            f"{len(equipped)}/{N_VEHICLES-1} "
            f"equipped followers, "
            f"minimum gap = {min_gap:.2f} m"
        )

    save_csv(
        OUTPUT / "summary.csv",
        summaries
    )

    print("\nTime-step sensitivity:")

    for penetration in PENETRATIONS:

        coarse, _, _ = run_case(
            penetration, dt=0.1
        )

        fine, _, _ = run_case(
            penetration, dt=0.05
        )

        last_id = N_VEHICLES - 1

        x_coarse = [
            r["position_m"]
            for r in coarse
            if r["vehicle_id"] == last_id
        ][-1]

        x_fine = [
            r["position_m"]
            for r in fine
            if r["vehicle_id"] == last_id
        ][-1]

        print(
            f"{penetration:.0%}: "
            f"final-position difference = "
            f"{abs(x_coarse-x_fine):.4f} m"
        )

    print(
        f"\nResults saved to: "
        f"{OUTPUT.resolve()}"
    )


if __name__ == "__main__":
    main()