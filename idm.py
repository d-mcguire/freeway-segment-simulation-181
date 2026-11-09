"""Intelligent Driver Model (Treiber, Hennecke & Helbing, 2000).

Internal units: seconds, meters, m/s, m/s^2.
Position is the FRONT bumper of a vehicle. Gap is bumper-to-bumper:
    gap = x_leader - length_leader - x_follower
"""
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class IDMParams:
    v0: float = 30.0      # desired speed (m/s)
    T: float = 1.5        # desired time gap (s)
    s0: float = 2.0       # minimum standstill gap (m)
    a: float = 1.0        # max acceleration (m/s^2)
    b: float = 1.5        # comfortable deceleration (m/s^2)
    delta: float = 4.0    # acceleration exponent
    length: float = 5.0   # vehicle length (m) -- ASSUMPTION, not in the Treiber table


@dataclass
class Vehicle:
    vehicle_id: int
    position_m: float
    speed_mps: float
    params: IDMParams = field(default_factory=IDMParams)
    lane_id: int = 0
    accel_mps2: float = 0.0


def desired_gap(v, dv, p):
    """s*(v, dv) = s0 + max(0, v*T + v*dv / (2*sqrt(a*b))).  dv = v - v_leader."""
    return p.s0 + max(0.0, v * p.T + v * dv / (2.0 * math.sqrt(p.a * p.b)))


def idm_accel(v, gap, dv, p):
    """IDM acceleration. gap=None means free road (no leader)."""
    free_term = 1.0 - (v / p.v0) ** p.delta
    if gap is None:
        return p.a * free_term
    if gap <= 0.0:
        raise ValueError(f"Non-positive gap ({gap:.3f} m): vehicles overlapped")
    return p.a * (free_term - (desired_gap(v, dv, p) / gap) ** 2)


def follower_accel(follower, leader):
    gap = leader.position_m - leader.params.length - follower.position_m
    dv = follower.speed_mps - leader.speed_mps
    return idm_accel(follower.speed_mps, gap, dv, follower.params)


def step_vehicle(veh, a, dt):
    """Ballistic update (constant acceleration over dt). Speed never goes negative."""
    v_new = veh.speed_mps + a * dt
    if v_new < 0.0:
        t_stop = veh.speed_mps / (-a)  # a < 0 here, since speed >= 0
        veh.position_m += veh.speed_mps * t_stop + 0.5 * a * t_stop ** 2
        veh.speed_mps = 0.0
    else:
        veh.position_m += veh.speed_mps * dt + 0.5 * a * dt ** 2
        veh.speed_mps = v_new
    veh.accel_mps2 = a


def step_platoon(vehicles, dt):
    """Advance a list of vehicles ordered leader-first (index 0 = front).
    All accelerations are computed from the SAME time-t state, then applied."""
    accs = []
    for i, veh in enumerate(vehicles):
        if i == 0:
            accs.append(idm_accel(veh.speed_mps, None, 0.0, veh.params))
        else:
            accs.append(follower_accel(veh, vehicles[i - 1]))
    for veh, a in zip(vehicles, accs):
        step_vehicle(veh, a, dt)


# ---- analytical references (used for verification) ----
def equilibrium_gap(v, p):
    """Steady-state bumper-to-bumper gap at speed v (v < v0)."""
    return (p.s0 + v * p.T) / math.sqrt(1.0 - (v / p.v0) ** p.delta)


def equilibrium_flow_veh_per_h(v, p):
    """Steady-state flow per lane at speed v, in veh/h."""
    return 3600.0 * v / (equilibrium_gap(v, p) + p.length)
