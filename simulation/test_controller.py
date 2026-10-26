"""Self-check of the control law against an ideal linear-spring wall (no PyBullet). Run: python test_controller.py"""
import numpy as np
from controller import Controller, Mode, Params

P = Params()
Z_WALL = 0.0                       # surface height; probe centre assumed to sit on the surface at z = 0


def simulate(mode, f_des=0.0, depth=0.03, T=6.0):
    """Operator ramps its reference `depth` below the wall; robot tip == p_c; wall force = Ke * penetration."""
    c = Controller(P if not f_des else Params(f_des=f_des), mode, [0, 0, 0.02])
    fn_log, z = [], 0.02
    for k in range(int(T / P.dt)):
        pen = max(0.0, Z_WALL - z)
        fe = np.array([0, 0, -P.Ke * pen])                   # robot pushes down on the wall
        u = np.array([0, 0, -0.01 if c.p_raw[2] > Z_WALL - depth else 0.0])
        o = c.step(u, fe, np.array([0, 0, z]), np.zeros(3))
        z = o["p_c"][2]
        fn_log.append(o["fn"])
    return np.array(fn_log), o


def test_unsaturated_force_follows_reference_depth():
    fn, _ = simulate(Mode(saturation=False))
    Keq = P.KP * P.Ke / (P.KP + P.Ke)
    assert abs(fn[-1] - Keq * 0.03) < 0.05 * Keq * 0.03, fn[-1]     # Eq. 8 steady state


def test_saturation_bounds_force_and_virtual_feedback_keeps_growing():
    fn, o = simulate(Mode(saturation=True))
    assert fn.max() < 1.1 * P.f_th, fn.max()                        # H2: bounded near f_th
    assert o["saturated"] and o["f_hn"] > 2 * P.f_th                # feedback still shows operator intent


def test_force_tracking_reaches_desired_force():
    fn, _ = simulate(Mode(force_track=True, saturation=True), f_des=5.0, depth=0.0)
    assert abs(fn[-1] - 5.0) < 0.3, fn[-1]


def test_no_phantom_cue_in_free_space():
    c = Controller(Params(f_des=5.0), Mode(force_track=True), [0, 0, 0.05])
    for _ in range(600):
        o = c.step(np.zeros(3), np.zeros(3), np.array([0, 0, 0.05]), np.zeros(3))
    assert o["a_vib"] < 0.02, o["a_vib"]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok  ", name)
