"""Observation, estimation and short-horizon forecasting.

Data flow (once per detector interval, e.g. every 30 s):

    true detector record  ->  ObservationPipeline  ->  StateEstimator  ->  FlowForecasters
    (simulator truth)         (adds defects)           (smoothed state)    (30/60 s ahead)

Only observations (never simulator truth) go to the estimator, forecasters and
controller. Truth is used afterwards, in attach_truth(), to score forecasts.

Record formats follow the Design Project Guide:
  observation : sensor_id, interval_start_s, interval_end_s, count, mean_speed_mps,
                quality_flag   (unavailable quantities are None, never silently 0)
  estimate    : issue_time_s, sensor_id, variable values + units, age, quality_flag
  forecast    : issue_time_s, target_time_s, segment_id, method, variable_name,
                units, predicted_value, quality_flag
"""
from collections import deque
from dataclasses import dataclass


# ---------------------------------------------------------------------------
# 1. Observation pipeline (with optional defects)
# ---------------------------------------------------------------------------
@dataclass
class ObservationDefects:
    missing_prob: float = 0.0     # probability that an interval report is lost
    delay_intervals: int = 0      # reports arrive this many intervals late
    count_noise_sd: float = 0.0   # additive Gaussian noise (veh) on the count


class ObservationPipeline:
    """Turns true detector records into (possibly degraded) observations."""

    def __init__(self, defects, rng):
        self.defects = defects
        self.rng = rng
        self._pending = []  # list of (deliver_time_s, observation)

    def ingest(self, sensor_id, record, interval_s):
        d = self.defects
        end_s = record["time_s"]
        obs = {
            "sensor_id": sensor_id,
            "interval_start_s": end_s - interval_s,
            "interval_end_s": end_s,
            "count": record["vehicle_count"],
            "mean_speed_mps": record["average_speed_mps"],          # time-mean
            "space_mean_speed_mps": record["space_mean_speed_mps"],  # harmonic mean
            "quality_flag": "ok" if record["vehicle_count"] > 0 else "no_vehicles",
        }

        if d.missing_prob > 0 and self.rng.random() < d.missing_prob:
            obs.update(count=None, mean_speed_mps=None,
                       space_mean_speed_mps=None, quality_flag="missing")
        elif d.count_noise_sd > 0:
            noisy = obs["count"] + self.rng.gauss(0.0, d.count_noise_sd)
            obs["count"] = max(0, round(noisy))
            obs["quality_flag"] = "noisy"

        deliver_s = end_s + d.delay_intervals * interval_s
        if d.delay_intervals > 0 and obs["quality_flag"] != "missing":
            obs["quality_flag"] = "delayed"
        self._pending.append((deliver_s, obs))

    def collect(self, time_s):
        """Return every observation that has been delivered by time_s."""
        ready = [o for t, o in self._pending if t <= time_s + 1e-9]
        self._pending = [(t, o) for t, o in self._pending if t > time_s + 1e-9]
        for o in ready:
            o["report_time_s"] = time_s
        return ready


# ---------------------------------------------------------------------------
# 2. State estimator (flow, speed, density per sensor)
# ---------------------------------------------------------------------------
class StateEstimator:
    """Exponentially smoothed flow and speed; density from k = q / v.

    density_veh_km = flow_veh_h / (space-mean speed in km/h)
    A missing report leaves the estimate unchanged and increases its age.
    """

    def __init__(self, sensor_ids, interval_s, v_free_mps=30.0,
                 smoothing=0.5, min_speed_mps=1.0):
        self.interval_s = interval_s
        self.v_free = v_free_mps
        self.smoothing = smoothing          # weight on the previous estimate
        self.min_speed = min_speed_mps
        self.state = {
            s: {"flow_vph": None, "speed_mps": None, "age": 0} for s in sensor_ids
        }

    def update(self, observations, time_s):
        updated = set()
        for obs in sorted(observations, key=lambda o: o["interval_end_s"]):
            st = self.state[obs["sensor_id"]]
            updated.add(obs["sensor_id"])
            if obs["count"] is None:          # explicit missing report
                st["age"] += 1
                continue

            flow = obs["count"] / self.interval_s * 3600.0
            speed = obs["space_mean_speed_mps"]
            if speed is None:                 # no vehicles -> speed unobservable
                speed = st["speed_mps"] if st["speed_mps"] is not None else self.v_free

            a = self.smoothing
            if st["flow_vph"] is None:
                st["flow_vph"], st["speed_mps"] = flow, speed
            else:
                st["flow_vph"] = a * st["flow_vph"] + (1 - a) * flow
                st["speed_mps"] = a * st["speed_mps"] + (1 - a) * speed
            st["age"] = 0

        for sensor_id, st in self.state.items():   # nothing arrived -> stale
            if sensor_id not in updated:
                st["age"] += 1

        return {sid: self._record(sid, time_s) for sid in self.state}

    def _record(self, sensor_id, time_s):
        st = self.state[sensor_id]
        if st["flow_vph"] is None:
            return {"issue_time_s": time_s, "sensor_id": sensor_id,
                    "flow_veh_h": None, "speed_mps": None, "density_veh_km": None,
                    "age_intervals": st["age"], "quality_flag": "no_data"}
        speed = max(st["speed_mps"], self.min_speed)
        density = st["flow_vph"] / (speed * 3.6)
        return {"issue_time_s": time_s, "sensor_id": sensor_id,
                "flow_veh_h": st["flow_vph"], "speed_mps": st["speed_mps"],
                "density_veh_km": density, "age_intervals": st["age"],
                "quality_flag": "ok" if st["age"] == 0 else "stale"}


# ---------------------------------------------------------------------------
# 3. Forecasters (interval flow, h intervals ahead)
# ---------------------------------------------------------------------------
class MovingAverageForecaster:
    """Baseline: mean of the last `window` observed interval flows."""
    name = "moving_average"

    def __init__(self, window=3):
        self.values = deque(maxlen=window)

    def update(self, flow_vph):
        self.values.append(flow_vph)

    def predict(self, horizon_intervals):
        return sum(self.values) / len(self.values) if self.values else None


class HoltForecaster:
    """Holt's linear (level + trend) exponential smoothing."""
    name = "holt_linear"

    def __init__(self, alpha=0.5, beta=0.3):
        self.alpha, self.beta = alpha, beta
        self.level = None
        self.trend = 0.0

    def update(self, flow_vph):
        if self.level is None:
            self.level = flow_vph
            return
        prev_level = self.level
        self.level = self.alpha * flow_vph + (1 - self.alpha) * (prev_level + self.trend)
        self.trend = self.beta * (self.level - prev_level) + (1 - self.beta) * self.trend

    def predict(self, horizon_intervals):
        if self.level is None:
            return None
        return max(0.0, self.level + horizon_intervals * self.trend)


class ForecastManager:
    """Runs both forecasters for every sensor and logs one record per method."""

    def __init__(self, sensor_ids, interval_s, horizon_intervals=2):
        self.interval_s = interval_s
        self.h = horizon_intervals
        self.models = {
            s: [MovingAverageForecaster(), HoltForecaster()] for s in sensor_ids
        }

    def update(self, observations, time_s):
        records = []
        for obs in sorted(observations, key=lambda o: o["interval_end_s"]):
            if obs["count"] is None:        # missing report: nothing to learn
                continue
            flow = obs["count"] / self.interval_s * 3600.0
            for model in self.models[obs["sensor_id"]]:
                model.update(flow)

        for sensor_id, models in self.models.items():
            for model in models:
                pred = model.predict(self.h)
                if pred is None:
                    continue
                records.append({
                    "issue_time_s": time_s,
                    "target_time_s": time_s + self.h * self.interval_s,
                    "segment_id": sensor_id,
                    "method": model.name,
                    "variable_name": "interval_flow",
                    "units": "veh/h",
                    "horizon_s": self.h * self.interval_s,
                    "predicted_value": pred,
                    "actual_value": None,
                    "quality_flag": "ok",
                })
        return records


# ---------------------------------------------------------------------------
# 4. Scoring against withheld truth
# ---------------------------------------------------------------------------
def attach_truth(forecasts, truth_flow):
    """truth_flow[(sensor_id, interval_end_s)] = true interval flow in veh/h."""
    for f in forecasts:
        key = (f["segment_id"], round(f["target_time_s"], 6))
        f["actual_value"] = truth_flow.get(key)
    return forecasts


def forecast_mae(forecasts, method, t_start_s, t_end_s, segment_id=None):
    """Mean absolute error (veh/h) for targets with t_start <= target < t_end."""
    errs = [
        abs(f["predicted_value"] - f["actual_value"])
        for f in forecasts
        if f["method"] == method
        and f["actual_value"] is not None
        and t_start_s <= f["target_time_s"] < t_end_s
        and (segment_id is None or f["segment_id"] == segment_id)
    ]
    return sum(errs) / len(errs) if errs else None
