"""Numerical checks for the unchanged idm.py implementation.

Run: python verify_idm.py
Uses only Python's standard library and idm.py.
"""
import math
from idm import (IDMParams, Vehicle, desired_gap, idm_accel,
                 equilibrium_gap, equilibrium_flow_veh_per_h,
                 follower_accel, step_vehicle)

HDV = IDMParams(v0=30.0, T=1.5, s0=2.0, a=1.0, b=1.5,
                delta=4.0, length=5.0)
CV = IDMParams(v0=30.0, T=0.7 * HDV.T, s0=2.0,
               a=1.5 * HDV.a, b=1.5, delta=4.0, length=5.0)
SPEED = 20.0


def test_parameters():
    assert math.isclose(CV.T, 1.05)
    assert math.isclose(CV.a, 1.5)
    assert HDV.v0 == CV.v0 == 30.0


def test_free_road():
    for p in (HDV, CV):
        assert idm_accel(10.0, None, 0.0, p) > 0
        assert math.isclose(idm_accel(p.v0, None, 0.0, p), 0.0,
                            abs_tol=1e-12)


def test_desired_gap():
    for p in (HDV, CV):
        assert math.isclose(desired_gap(SPEED, 0.0, p),
                            p.s0 + SPEED * p.T)
        assert desired_gap(SPEED, 5.0, p) > desired_gap(SPEED, 0.0, p)


def test_equilibrium():
    for p in (HDV, CV):
        s = equilibrium_gap(SPEED, p)
        assert math.isclose(idm_accel(SPEED, s, 0.0, p), 0.0,
                            abs_tol=1e-10)
        expected_q = 3600.0 * SPEED / (s + p.length)
        assert math.isclose(equilibrium_flow_veh_per_h(SPEED, p),
                            expected_q)


def test_braking_response():
    for p in (HDV, CV):
        normal = idm_accel(SPEED, 35.0, 0.0, p)
        closing = idm_accel(SPEED, 35.0, 5.0, p)
        close = idm_accel(SPEED, 10.0, 5.0, p)
        assert closing < normal
        assert close < closing


def test_nonpositive_gap_detection():
    try:
        idm_accel(SPEED, 0.0, 0.0, HDV)
    except ValueError:
        pass
    else:
        raise AssertionError('Zero gap was not detected')


def test_stopping_rule():
    car = Vehicle(0, 0.0, 2.0, HDV)
    step_vehicle(car, -4.0, 1.0)
    assert car.speed_mps == 0.0
    assert math.isclose(car.position_m, 0.5, abs_tol=1e-12)


def equilibrium_platoon(dt, duration_s=20.0, n=8):
    """Prescribed leader at 20 m/s; all followers start in equilibrium."""
    s = equilibrium_gap(SPEED, HDV) + HDV.length
    cars = [Vehicle(i, (n - 1 - i) * s, SPEED, HDV) for i in range(n)]
    min_gap = float('inf')
    for _ in range(round(duration_s / dt)):
        accs = [0.0] + [follower_accel(cars[i], cars[i - 1])
                        for i in range(1, n)]
        for car, a in zip(cars, accs):
            step_vehicle(car, a, dt)
        for lead, follow in zip(cars, cars[1:]):
            gap = lead.position_m - lead.params.length - follow.position_m
            min_gap = min(min_gap, gap)
            assert gap > 0, f'Overlap at dt={dt}'
    return cars, min_gap


def test_equilibrium_platoon():
    for dt in (0.1, 0.05):
        cars, min_gap = equilibrium_platoon(dt)
        assert min_gap > 0
        for car in cars:
            assert abs(car.speed_mps - SPEED) < 1e-8
    print('  Equilibrium platoon stable at dt=0.1 and dt=0.05 s')


def main():
    tests = [test_parameters, test_free_road, test_desired_gap,
             test_equilibrium, test_braking_response,
             test_nonpositive_gap_detection, test_stopping_rule,
             test_equilibrium_platoon]
    for test in tests:
        test()
        print(f'PASS {test.__name__}')
    print('\nAll IDM checks passed. This is not a full collision-safety proof.')
    print(f'HDV equilibrium gap at 20 m/s: {equilibrium_gap(SPEED, HDV):.3f} m')
    print(f'CV equilibrium gap at 20 m/s: {equilibrium_gap(SPEED, CV):.3f} m')


if __name__ == '__main__':
    main()
