from dataclasses import dataclass

from idm import Vehicle, IDMParams, follower_accel, idm_accel, step_vehicle
from detectors import create_detectors


ROAD_LENGTH_M = 3.0 * 1609.344
RAMP_POSITION_M = 2.5 * 1609.344

DT = 0.1
DURATION_S = 600.0

MAINLINE_DEMAND_VPH = 2000.0
RAMP_DEMAND_VPH = 400.0
RAMP_METER_VPH = 600.0

ENTRY_SPEED_MPS = 20.0
RAMP_ENTRY_SPEED_MPS = 20.0

MIN_ENTRY_GAP_M = 10.0
MIN_MERGE_GAP_M = 10.0
MAX_ENTRY_BRAKING_MPS2 = 3.0
MAX_MERGE_BRAKING_MPS2 = 3.0

HDV = IDMParams()


@dataclass
class SimulationStats:
    mainline_requested: int = 0
    mainline_admitted: int = 0
    ramp_requested: int = 0
    ramp_merged: int = 0
    exits: int = 0

    entry_blocked_gap: int = 0
    entry_blocked_braking: int = 0

    ramp_blocked_front: int = 0
    ramp_blocked_rear: int = 0
    ramp_blocked_braking: int = 0
    ramp_meter_waiting: int = 0


def gap_between(follower, leader):
    return (
        leader.position_m
        - leader.params.length
        - follower.position_m
    )


def create_mainline_vehicle(vehicle_id, vehicles):
    speed = ENTRY_SPEED_MPS

    if vehicles:
        rear_vehicle = min(
            vehicles,
            key=lambda car: car.position_m
        )
        speed = min(ENTRY_SPEED_MPS, rear_vehicle.speed_mps)

    return Vehicle(
        vehicle_id=vehicle_id,
        position_m=0.0,
        speed_mps=speed,
        params=HDV
    )


def can_enter_mainline(vehicles, new_vehicle, stats):
    if not vehicles:
        return True

    rear_vehicle = min(
        vehicles,
        key=lambda car: car.position_m
    )

    gap = gap_between(new_vehicle, rear_vehicle)

    if gap < MIN_ENTRY_GAP_M:
        stats.entry_blocked_gap += 1
        return False

    acceleration = follower_accel(
        new_vehicle,
        rear_vehicle
    )

    if acceleration < -MAX_ENTRY_BRAKING_MPS2:
        stats.entry_blocked_braking += 1
        return False

    return True


def check_ramp_merge(vehicles, new_vehicle, stats):
    leader = None
    follower = None

    for car in vehicles:
        if car.position_m > RAMP_POSITION_M:
            if leader is None or car.position_m < leader.position_m:
                leader = car
        else:
            if follower is None or car.position_m > follower.position_m:
                follower = car

    if leader is not None:
        front_gap = gap_between(new_vehicle, leader)

        if front_gap < MIN_MERGE_GAP_M:
            stats.ramp_blocked_front += 1
            return False

        merge_acceleration = follower_accel(
            new_vehicle,
            leader
        )

        if merge_acceleration < -MAX_MERGE_BRAKING_MPS2:
            stats.ramp_blocked_braking += 1
            return False

    if follower is not None:
        rear_gap = gap_between(follower, new_vehicle)

        if rear_gap < MIN_MERGE_GAP_M:
            stats.ramp_blocked_rear += 1
            return False

        follower_acceleration = follower_accel(
            follower,
            new_vehicle
        )

        if follower_acceleration < -MAX_MERGE_BRAKING_MPS2:
            stats.ramp_blocked_braking += 1
            return False

    return True


def minimum_gap(vehicles):
    if len(vehicles) < 2:
        return float("inf")

    ordered = sorted(
        vehicles,
        key=lambda car: car.position_m,
        reverse=True
    )

    return min(
        gap_between(ordered[i], ordered[i - 1])
        for i in range(1, len(ordered))
    )


def simulate():
    vehicles = []
    stats = SimulationStats()
    detectors = create_detectors()

    mainline_queue = 0
    ramp_queue = 0
    next_id = 0

    mainline_arrival_credit = 0.0
    ramp_arrival_credit = 0.0
    meter_credit = 0.0

    smallest_gap = float("inf")
    n_steps = round(DURATION_S / DT)

    for step in range(n_steps):
        mainline_arrival_credit += MAINLINE_DEMAND_VPH * DT / 3600.0
        ramp_arrival_credit += RAMP_DEMAND_VPH * DT / 3600.0

        meter_credit = min(
            1.0,
            meter_credit + RAMP_METER_VPH * DT / 3600.0
        )

        while mainline_arrival_credit >= 1.0:
            mainline_queue += 1
            stats.mainline_requested += 1
            mainline_arrival_credit -= 1.0

        while ramp_arrival_credit >= 1.0:
            ramp_queue += 1
            stats.ramp_requested += 1
            ramp_arrival_credit -= 1.0

        if mainline_queue > 0:
            new_vehicle = create_mainline_vehicle(
                next_id,
                vehicles
            )

            if can_enter_mainline(
                vehicles,
                new_vehicle,
                stats
            ):
                vehicles.append(new_vehicle)
                mainline_queue -= 1
                stats.mainline_admitted += 1
                next_id += 1

        if ramp_queue > 0:
            if meter_credit >= 1.0:
                new_vehicle = Vehicle(
                    vehicle_id=next_id,
                    position_m=RAMP_POSITION_M,
                    speed_mps=RAMP_ENTRY_SPEED_MPS,
                    params=HDV
                )

                if check_ramp_merge(
                    vehicles,
                    new_vehicle,
                    stats
                ):
                    vehicles.append(new_vehicle)
                    ramp_queue -= 1
                    stats.ramp_merged += 1
                    next_id += 1
                    meter_credit -= 1.0
            else:
                stats.ramp_meter_waiting += 1

        vehicles.sort(
            key=lambda car: car.position_m,
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

        previous_positions = {
            car.vehicle_id: car.position_m
            for car in vehicles
        }

        for car, acceleration in zip(vehicles, accelerations):
            step_vehicle(car, acceleration, DT)

        current_time = (step + 1) * DT

        for detector in detectors:
            detector.update(
                previous_positions,
                vehicles,
                DT
            )

            interval_steps = round(detector.interval_s / DT)

            if (step + 1) % interval_steps == 0:
                detector.record_interval(current_time)

        before = len(vehicles)

        vehicles = [
            car for car in vehicles
            if car.position_m < ROAD_LENGTH_M
        ]

        stats.exits += before - len(vehicles)

        current_gap = minimum_gap(vehicles)
        smallest_gap = min(smallest_gap, current_gap)

        if current_gap <= 0.0:
            raise RuntimeError(
                f"Vehicle overlap detected at t={current_time:.1f} s"
            )

    mainline_conservation = (
        stats.mainline_requested
        == stats.mainline_admitted + mainline_queue
    )

    ramp_conservation = (
        stats.ramp_requested
        == stats.ramp_merged + ramp_queue
    )

    total_conservation = (
        stats.mainline_admitted + stats.ramp_merged
        == stats.exits + len(vehicles)
    )

    print("Simulation complete")
    print(f"Freeway length: {ROAD_LENGTH_M:.1f} m")
    print(f"Ramp location: {RAMP_POSITION_M:.1f} m")
    print(f"Mainline demand: {MAINLINE_DEMAND_VPH:.0f} veh/h")
    print(f"Ramp demand: {RAMP_DEMAND_VPH:.0f} veh/h")
    print(f"Ramp meter limit: {RAMP_METER_VPH:.0f} veh/h")

    print("\nMainline results")
    print(f"Mainline requested: {stats.mainline_requested}")
    print(f"Mainline admitted: {stats.mainline_admitted}")
    print(f"Mainline upstream queue: {mainline_queue}")
    print(
        f"Actual mainline entry rate: "
        f"{stats.mainline_admitted / DURATION_S * 3600:.1f} veh/h"
    )

    print("\nMainline entry diagnostics")
    print(f"Blocked by entry gap: {stats.entry_blocked_gap}")
    print(f"Blocked by braking: {stats.entry_blocked_braking}")

    print("\nRamp results")
    print(f"Ramp requested: {stats.ramp_requested}")
    print(f"Ramp vehicles merged: {stats.ramp_merged}")
    print(f"Ramp queue remaining: {ramp_queue}")

    print("\nRamp merging diagnostics")
    print(f"Blocked by front gap: {stats.ramp_blocked_front}")
    print(f"Blocked by rear gap: {stats.ramp_blocked_rear}")
    print(f"Blocked by braking: {stats.ramp_blocked_braking}")
    print(f"Waiting for meter: {stats.ramp_meter_waiting}")

    print("\nTraffic results")
    print(f"Total exits: {stats.exits}")
    print(f"Vehicles remaining: {len(vehicles)}")

    if smallest_gap != float("inf"):
        print(f"Minimum vehicle gap: {smallest_gap:.2f} m")
    else:
        print("Minimum vehicle gap: N/A")

    print("\nVerification")
    print(
        "Mainline conservation check:",
        "PASSED" if mainline_conservation else "FAILED"
    )
    print(
        "Ramp conservation check:",
        "PASSED" if ramp_conservation else "FAILED"
    )
    print(
        "Total vehicle conservation check:",
        "PASSED" if total_conservation else "FAILED"
    )
    print(
        "Vehicle overlap check:",
        "PASSED" if smallest_gap > 0 else "FAILED"
    )

    assert mainline_conservation
    assert ramp_conservation
    assert total_conservation
    assert smallest_gap > 0

    print("\nVirtual detector results")

    for detector in detectors:
        detector.print_summary(DURATION_S)


if __name__ == "__main__":
    simulate()
