
from dataclasses import dataclass
import random

from idm import (
    Vehicle,
    IDMParams,
    follower_accel,
    idm_accel,
    step_vehicle,
)

ROAD_LENGTH_M = 3.0 * 1609.344
DT = 0.1
DURATION_S = 600.0

MAINLINE_DEMAND_VPH = 2000.0
ENTRY_SPEED_MPS = 20.0
MINIMUM_ENTRY_GAP_M = 10.0
MAX_ENTRY_BRAKING_MPS2 = -3.0

HDV = IDMParams()


@dataclass
class SimulationStats:
    arrivals: int = 0
    exits: int = 0


def can_enter(vehicles, new_vehicle):
    if not vehicles:
        return True

    rear_vehicle = min(vehicles, key=lambda v: v.position_m)

    gap = (
        rear_vehicle.position_m
        - rear_vehicle.params.length
        - new_vehicle.position_m
    )

    if gap < MINIMUM_ENTRY_GAP_M:
        return False

    relative_speed = (
        new_vehicle.speed_mps - rear_vehicle.speed_mps
    )

    acceleration = idm_accel(
        new_vehicle.speed_mps,
        gap,
        relative_speed,
        new_vehicle.params
    )

    return acceleration >= MAX_ENTRY_BRAKING_MPS2


def check_vehicle_gaps(vehicles, time_s):
    vehicles.sort(
        key=lambda v: v.position_m,
        reverse=True
    )

    minimum_gap = float("inf")

    for i in range(1, len(vehicles)):
        leader = vehicles[i - 1]
        follower = vehicles[i]

        gap = (
            leader.position_m
            - leader.params.length
            - follower.position_m
        )

        minimum_gap = min(minimum_gap, gap)

        if gap <= 0:
            raise RuntimeError(
                f"Vehicle overlap at t={time_s:.1f} s "
                f"between vehicles {leader.vehicle_id} "
                f"and {follower.vehicle_id}"
            )

    return minimum_gap


def simulate():
    rng = random.Random(181)

    vehicles = []
    stats = SimulationStats()

    next_id = 0
    pending_arrivals = 0
    requested_arrivals = 0
    minimum_gap_recorded = float("inf")

    n_steps = round(DURATION_S / DT)

    arrival_probability = (
        MAINLINE_DEMAND_VPH / 3600.0 * DT
    )

    for step in range(n_steps):
        time_s = step * DT

        if rng.random() < arrival_probability:
            pending_arrivals += 1
            requested_arrivals += 1

        if pending_arrivals > 0:
            new_vehicle = Vehicle(
                vehicle_id=next_id,
                position_m=0.0,
                speed_mps=ENTRY_SPEED_MPS,
                params=HDV,
            )

            if can_enter(vehicles, new_vehicle):
                vehicles.append(new_vehicle)
                pending_arrivals -= 1
                next_id += 1
                stats.arrivals += 1

        vehicles.sort(
            key=lambda v: v.position_m,
            reverse=True
        )

        accelerations = []

        for i, car in enumerate(vehicles):
            if i == 0:
                acceleration = idm_accel(
                    car.speed_mps,
                    None,
                    0.0,
                    car.params
                )
            else:
                acceleration = follower_accel(
                    car,
                    vehicles[i - 1]
                )

            accelerations.append(acceleration)

        for car, acceleration in zip(vehicles, accelerations):
            step_vehicle(car, acceleration, DT)

        minimum_gap = check_vehicle_gaps(
            vehicles,
            time_s + DT
        )

        minimum_gap_recorded = min(
            minimum_gap_recorded,
            minimum_gap
        )

        before = len(vehicles)

        vehicles = [
            car for car in vehicles
            if car.position_m < ROAD_LENGTH_M
        ]

        stats.exits += before - len(vehicles)

    requested_rate = (
        requested_arrivals / DURATION_S * 3600.0
    )

    entry_rate = (
        stats.arrivals / DURATION_S * 3600.0
    )

    exit_rate = (
        stats.exits / DURATION_S * 3600.0
    )

    assert (
        requested_arrivals
        == stats.arrivals + pending_arrivals
    )

    assert (
        stats.arrivals
        == stats.exits + len(vehicles)
    )

    print("Simulation complete")
    print(f"Traffic demand: {MAINLINE_DEMAND_VPH:.0f} veh/h")
    print(f"Requested arrivals: {requested_arrivals}")
    print(f"Arrivals admitted: {stats.arrivals}")
    print(f"Exits: {stats.exits}")
    print(f"Vehicles remaining: {len(vehicles)}")
    print(f"Upstream queue: {pending_arrivals}")
    print(f"Requested arrival rate: {requested_rate:.1f} veh/h")
    print(f"Actual entry rate: {entry_rate:.1f} veh/h")
    print(f"Exit rate: {exit_rate:.1f} veh/h")

    if minimum_gap_recorded != float("inf"):
        print(
            f"Minimum vehicle gap: "
            f"{minimum_gap_recorded:.2f} m"
        )

    print("Vehicle conservation check: PASSED")
    print("Vehicle overlap check: PASSED")


if __name__ == "__main__":
    simulate()
