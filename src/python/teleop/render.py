#!/usr/bin/env python3
"""Render the scenario offscreen for the report: scenario C, scripted operator, after 4 s of the trace.

  python -m teleop.render                      # -> figures/sim_scene.png
  python -m teleop.render --scenario X --t 5   # any scenario, any time
"""
import argparse
from dataclasses import replace

import numpy as np
import pybullet as p

from . import config, run
from .controller import Controller, Params
from .scene import PROBE_R, Scene


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="C")
    ap.add_argument("--t", type=float, default=4.0, help="simulated time at which to take the picture [s]")
    ap.add_argument("--out", default=str(config.FIGURES / "sim_scene.png"))
    a = ap.parse_args()
    surface, mode, f_des, haptic = run.SCENARIOS[a.scenario]
    P = replace(Params(), f_des=f_des)
    sc = Scene(gui=False, surface=surface)
    p0 = np.array([run.X0, 0, sc.surface_height_normal(run.X0)[0] + PROBE_R + 0.03])
    for _ in range(240):
        sc.command_tip(p0)
        sc.step()
    ctrl = Controller(P, mode, p0)
    op = run.Scripted(sc, guided=mode.force_track, reactive=False, haptic=haptic)
    obs, t = None, 0.0
    while t < a.t:
        u, _ = op(t, obs)
        tip, vel = sc.tip()
        fe, n, _ = sc.contact_wrench()
        obs = ctrl.step(u, fe, tip, vel, n)
        sc.command_tip(obs["p_c"])
        sc.step()
        t += P.dt
    view = p.computeViewMatrixFromYawPitchRoll([0.5, 0, sc.surface_top + 0.1], 1.25, 40, -22, 0, 2)
    proj = p.computeProjectionMatrixFOV(45, 4 / 3, 0.05, 5)
    w, h, rgb, _, _ = p.getCameraImage(1200, 900, view, proj, renderer=p.ER_TINY_RENDERER,
                                       lightDirection=[1, 1, 2], shadow=1)
    sc.close()
    import matplotlib.image as mpimg
    config.FIGURES.mkdir(parents=True, exist_ok=True)
    mpimg.imsave(a.out, np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8))
    print(f"t={t:.2f} s  fn={obs['fn']:.1f} N  ->", a.out)


if __name__ == "__main__":
    main()
