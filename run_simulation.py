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
