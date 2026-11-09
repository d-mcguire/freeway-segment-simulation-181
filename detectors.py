from dataclasses import dataclass, field


@dataclass
class Detector:
    name: str
    position_m: float
    interval_s: float = 30.0

    total_crossings: int = 0
    interval_crossings: int = 0
    interval_speed_sum: float = 0.0
    total_speed_sum: float = 0.0
    records: list = field(default_factory=list)

    def update(self, previous_positions, vehicles, dt):
        for vehicle in vehicles:
            previous_x = previous_positions.get(vehicle.vehicle_id)

            if previous_x is None:
                continue

            if previous_x < self.position_m <= vehicle.position_m:
                distance_moved = vehicle.position_m - previous_x

                if distance_moved > 0:
                    crossing_speed = distance_moved / dt
                else:
                    crossing_speed = vehicle.speed_mps

                self.total_crossings += 1
                self.interval_crossings += 1

                self.interval_speed_sum += crossing_speed
                self.total_speed_sum += crossing_speed

    def record_interval(self, time_s, duration_s=None):
        if duration_s is None:
            duration_s = self.interval_s

        flow = self.interval_crossings / duration_s * 3600.0

        if self.interval_crossings > 0:
            average_speed = (
                self.interval_speed_sum / self.interval_crossings
            )
        else:
            average_speed = None

        self.records.append({
            "time_s": time_s,
            "flow_veh_h": flow,
            "average_speed_mps": average_speed,
            "vehicle_count": self.interval_crossings
        })

        self.interval_crossings = 0
        self.interval_speed_sum = 0.0

    def print_summary(self, duration_s):
        average_flow = (
            self.total_crossings / duration_s * 3600.0
        )

        if self.total_crossings > 0:
            average_speed = (
                self.total_speed_sum / self.total_crossings
            )
        else:
            average_speed = None

        print(f"\n{self.name}")
        print(f"Position: {self.position_m:.1f} m")
        print(f"Total crossings: {self.total_crossings}")
        print(f"Average flow: {average_flow:.1f} veh/h")

        if average_speed is not None:
            print(f"Average speed: {average_speed:.2f} m/s")
            print(f"Average speed: {average_speed * 2.23694:.2f} mph")
        else:
            print("Average speed: N/A")


def create_detectors():
    mile = 1609.344

    return [
        Detector("Detector 1 - Mile 1.0", 1.0 * mile),
        Detector("Detector 2 - Mile 2.0", 2.0 * mile),
        Detector("Detector 3 - Mile 2.4", 2.4 * mile),
        Detector("Detector 4 - Mile 2.8", 2.8 * mile)
    ]
