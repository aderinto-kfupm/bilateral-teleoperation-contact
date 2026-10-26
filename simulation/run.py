#!/usr/bin/env python3
"""Run the report's experiments (Sec. VII) in PyBullet and log everything needed to judge them.

  python run.py --scenario C                       # one scenario, headless, scripted+reactive operator
  python run.py --scenario all                     # baseline, A, B, C + curved-surface extension (X*)
  python run.py --scenario C --gui                 # watch it (real-time, force arrow + cue overlay)
  python run.py --scenario C --operator dualsense --gui   # you drive with the PS5 controller

Outputs go to runs/<stamp>/: <scenario>.csv (per-step trace), <scenario>.png (dashboard),
summary.csv + summary.json (metrics of Sec. VII-C), comparison.png (when several scenarios ran).
"""
import argparse
import csv
import json
import os
import time
from dataclasses import replace
import numpy as np

from controller import Controller, Mode, Params
from scene import PROBE_R, Scene

X0, X1 = 0.45, 0.65           # traced path along x on the surface (y = 0)
F_FAIL = 20.0                 # peak normal force that counts as a failed (unsafe) run [N]
LOSS_FAIL_S = 1.0             # continuous contact loss during the trace that counts as failure [s]
TIMEOUT_S = 25.0

# name: (surface, Mode, f_des, haptic cues shown to operator)
SCENARIOS = {
    "baseline": ("plane", Mode(admittance=False, virtual_fb=False), 0.0, False),
    "A":        ("plane", Mode(virtual_fb=False), 0.0, True),
    "B":        ("plane", Mode(virtual_fb=True), 0.0, True),
    "C":        ("plane", Mode(saturation=True, virtual_fb=True), 0.0, True),
    "X":        ("curved", Mode(saturation=True, force_track=True, normal="est"), 5.0, True),
    "Xfixed":   ("curved", Mode(saturation=True, force_track=True, normal="fixed"), 5.0, True),
    "Xoracle":  ("curved", Mode(saturation=True, force_track=True, normal="oracle"), 5.0, True),
}
PLANAR, CURVED = ["baseline", "A", "B", "C"], ["X", "Xfixed", "Xoracle"]


# ------------------------------------------------------------------ operators
class Scripted:
    """Open-loop operator: approach, then slide along +x while (optionally) over-pressing into the surface.
    'reactive' adds a human-like reflex: pressing eases off as the haptic cue grows (reaction delay + lag)."""

    def __init__(self, scene, guided, reactive, haptic, seed=0, v_app=0.04, v_t=0.03, v_push=0.01, push_depth=0.03):
        self.s, self.guided, self.reactive, self.haptic = scene, guided, reactive, haptic
        self.rng = np.random.default_rng(seed)
        self.v_app, self.v_t, self.v_push, self.push_depth = v_app, v_t, v_push, push_depth
        self.cue_buf = [0.0] * 36          # ~150 ms reaction delay at 240 Hz
        self.un = 0.0
        self.phase = "approach"

    def __call__(self, t, obs):
        cue = max(obs["a_vib"], obs["a_trig"]) if (self.haptic and obs) else 0.0
        self.cue_buf.append(cue)
        cue_d = self.cue_buf.pop(0)
        if self.phase == "approach" and obs and obs["in_contact"]:
            self.phase = "trace"
        if self.phase == "approach":
            return np.array([0.0, 0.0, -self.v_app]), self.phase
        if obs["p_raw"][0] >= X1:
            return np.zeros(3), "done"
        ut = self.v_t + self.rng.normal(0, 0.004)          # hand tremor on the tangential command
        rest_z = self.s.surface_height_normal(obs["p_raw"][0])[0] + PROBE_R
        pressing = 0.0 if self.guided else (self.v_push if rest_z - obs["p_raw"][2] < self.push_depth else 0.0)
        target = -pressing * (1 - 2 * cue_d) if self.reactive else -pressing
        self.un += 0.05 * (target - self.un)               # ~80 ms neuromuscular lag
        return np.array([ut, 0.0, self.un]), self.phase


class DualSense:
    """Left stick = tangential x/y, right stick y = z. Rumble = cue; trigger resistance is log/overlay only."""

    def __init__(self, index=0, speed=0.06, clutch_button=None):
        import pygame
        pygame.init()
        pygame.joystick.init()
        self.pg, self.speed, self.clutch = pygame, speed, clutch_button
        self.js = pygame.joystick.Joystick(index)
        self.js.init()
        self.phase, self.last_rumble = "manual", 0.0

    def __call__(self, t, obs):
        self.pg.event.pump()
        ax = lambda i: (lambda v: 0.0 if abs(v) < 0.08 else v)(self.js.get_axis(i))
        engaged = self.clutch is None or self.js.get_button(self.clutch)
        u = np.array([ax(0), -ax(1), -ax(3)]) * self.speed if engaged else np.zeros(3)
        if obs and t - self.last_rumble > 0.03:
            self.js.rumble(obs["a_vib"], min(1.0, 0.6 * obs["a_trig"]), 60)
            self.last_rumble = t
        return u, self.phase


# -------------------------------------------------------------------- one run
def run_scenario(name, args):
    surface, mode, f_des, haptic = SCENARIOS[name]
    P = replace(Params(), f_des=f_des)
    sc = Scene(gui=args.gui, surface=surface)
    z0 = sc.surface_height_normal(X0)[0] + PROBE_R + 0.03
    p0 = np.array([X0, 0.0, z0])
    for _ in range(240):
        sc.command_tip(p0)
        sc.step()
    ctrl = Controller(P, mode, p0)
    op = (DualSense(clutch_button=args.clutch_button) if args.operator == "dualsense" else
          Scripted(sc, guided=mode.force_track, reactive=args.operator == "reactive", haptic=haptic, seed=args.seed))
    overlay = Overlay(sc, name) if args.gui else None

    rows, obs, t, done_t = [], None, 0.0, None
    try:
        while t < TIMEOUT_S:
            t0 = time.perf_counter()
            u, phase = op(t, obs)
            tip, vel = sc.tip()
            fe, n_true, npts = sc.contact_wrench()
            obs = ctrl.step(u, fe, tip, vel, n_true)
            info = sc.command_tip(obs["p_c"])
            t1 = time.perf_counter()
            sc.step()
            t += P.dt
            z_s, n_gt = sc.surface_height_normal(tip[0])
            n_gt = n_true if n_true is not None else n_gt
            dist = (np.linalg.norm(tip - sc.cyl_c) - sc.cyl_R - PROBE_R) if surface == "curved" else tip[2] - z_s - PROBE_R
            ang = lambda a, b: float(np.degrees(np.arccos(np.clip(a @ b, -1, 1))))
            rows.append(dict(
                t=t, phase=phase, **{f"u_{k}": u[i] for i, k in enumerate("xyz")},
                **{f"{key}_{k}": obs[key][i] for key in ("p_h", "p_d", "p_c") for i, k in enumerate("xyz")},
                **{f"tip_{k}": tip[i] for i, k in enumerate("xyz")},
                **{f"fe_{k}": fe[i] for i, k in enumerate("xyz")},
                fn=obs["fn"], f_hn=obs["f_hn"], a_vib=obs["a_vib"], a_trig=obs["a_trig"],
                saturated=int(obs["saturated"]), in_contact=int(obs["in_contact"]), n_pts=npts,
                n_used_err_deg=ang(obs["n"], n_gt), n_est_err_deg=ang(obs["n_est"], n_gt),
                dist_to_surface=dist, near_limit=int(info["near_joint_limit"]), ik_err=info["ik_err"],
                ctrl_ms=(t1 - t0) * 1e3))
            if overlay and len(rows) % 6 == 0:
                overlay.update(tip, fe, obs, phase)
            if args.gui:
                time.sleep(max(0.0, P.dt - (time.perf_counter() - t0)))
            if phase == "done":
                done_t = done_t or t
                if t - done_t > 0.5:
                    break
    except KeyboardInterrupt:
        print("interrupted: saving what was logged")
    sc.close()
    return rows, P


class Overlay:
    """GUI observability: force arrow, reference/command markers, text readout, path line."""

    def __init__(self, scene, name):
        import pybullet as p
        self.p, self.s, self.name, self.ids = p, scene, name, {}
        pts = [[x, 0, scene.surface_height_normal(x)[0] + 0.002] for x in np.linspace(X0, X1, 40)]
        for a, b in zip(pts[:-1], pts[1:]):
            p.addUserDebugLine(a, b, [1, 1, 0], 3)
        p.resetDebugVisualizerCamera(0.7, 35, -30, [0.55, 0, scene.surface_top])

    def _line(self, key, a, b, rgb):
        self.ids[key] = self.p.addUserDebugLine(a, b, rgb, 4, replaceItemUniqueId=self.ids.get(key, -1))

    def update(self, tip, fe, obs, phase):
        self._line("force", tip.tolist(), (tip - fe * 0.01).tolist(), [min(1, obs["fn"] / 12), 0.2, 0.2])
        for key, rgb in (("p_h", [0, 1, 0]), ("p_c", [0, 0.4, 1])):
            self._line("m_" + key, (obs[key] - [0.01, 0, 0]).tolist(), (obs[key] + [0.01, 0, 0]).tolist(), rgb)
        txt = (f"{self.name} [{phase}]  fn={obs['fn']:5.1f}N  vib={obs['a_vib']:.2f}  trig={obs['a_trig']:.2f}"
               f"  sat={'ON' if obs['saturated'] else 'off'}   green=p_h(operator) blue=p_c(robot)")
        self.ids["txt"] = self.p.addUserDebugText(txt, [0.30, -0.3, self.s.surface_top + 0.25], [0, 0, 0], 1.2,
                                                  replaceItemUniqueId=self.ids.get("txt", -1))


# ------------------------------------------------------------ metrics + plots
def metrics(name, rows, P):
    c = lambda k: np.array([r[k] for r in rows], float)
    dt = P.dt
    tr = np.array([r["phase"] in ("trace", "manual") for r in rows])
    if not tr.any():
        return dict(scenario=name, success=False, fail_reason="never reached trace phase")
    fn, contact, sat = c("fn")[tr], c("in_contact")[tr].astype(bool), c("saturated")[tr]
    a_vib = c("a_vib")[tr]
    # contact loss = contact false after first contact, before the trace ended
    first = np.argmax(contact) if contact.any() else len(contact)
    lost = ~contact[first:]
    edges = np.diff(np.r_[0, lost.astype(int), 0])
    runs = np.where(edges == -1)[0] - np.where(edges == 1)[0]
    max_loss_s = float(runs.max() * dt) if len(runs) else 0.0
    done = any(r["phase"] == "done" for r in rows)
    reasons = []
    if not done: reasons.append("timeout")
    if fn.max() > F_FAIL: reasons.append(f"peak force>{F_FAIL:.0f}N")
    if max_loss_s > LOSS_FAIL_S: reasons.append("prolonged contact loss")
    if c("near_limit").any(): reasons.append("joint limit")
    x = c("tip_x")[tr]
    covered = np.unique(np.clip(np.digitize(x[contact], np.linspace(X0, X1, 21)), 1, 20))
    dist = c("dist_to_surface")[tr]
    path_err = np.sqrt(np.mean(c("tip_y")[tr] ** 2 + dist ** 2))
    d = lambda v: np.diff(v) / dt
    cm = contact & (c("n_pts")[tr] > 0)
    nerr = c("n_used_err_deg")[tr][cm]
    r2 = lambda v: None if v is None else round(float(v), 2)
    return dict(
        scenario=name, success=not reasons, fail_reason=";".join(reasons),
        completion_time_s=r2(tr.sum() * dt),
        fn_peak=r2(fn.max()), fn_mean=r2(fn[contact].mean() if contact.any() else 0),
        fn_std=r2(fn[contact].std() if contact.any() else 0),
        fn_rms_err=r2(np.sqrt(np.mean((fn[contact] - P.f_des) ** 2))) if P.f_des > 0 and contact.any() else None,
        t_above_fth_s=r2((fn > P.f_th).sum() * dt), t_above_fstop_s=r2((fn > P.f_stop).sum() * dt),
        contact_loss_count=int(len(runs)), contact_loss_total_s=r2(lost.sum() * dt), contact_loss_max_s=r2(max_loss_s),
        path_err_rms_mm=r2(path_err * 1e3), coverage_pct=r2(100 * len(covered) / 20),
        cue_hf_rms=r2(np.sqrt(np.mean(d(a_vib) ** 2))),                       # H1: feedback variation [1/s]
        cmd_osc_rms=r2(np.sqrt(np.mean(d(c("u_z")[tr]) ** 2))),               # H1: operator-command oscillation [m/s^2]
        saturation_activity_pct=r2(100 * sat.mean()),
        normal_err_mean_deg=r2(nerr.mean()) if len(nerr) else None,
        normal_err_max_deg=r2(nerr.max()) if len(nerr) else None,
        ctrl_ms_mean=r2(c("ctrl_ms").mean()), ctrl_ms_max=r2(c("ctrl_ms").max()),
        ik_err_max_mm=r2(c("ik_err").max() * 1e3))


def dashboard(name, rows, P, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    c = lambda k: np.array([r[k] for r in rows], float)
    t = c("t")
    fig, ax = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
    ax[0].plot(t, c("fn"), label="measured normal force (filtered)")
    ax[0].plot(t, c("f_hn"), "--", label="virtual feedback |f_h,n|")
    for v, lab, col in ((P.f_th, "f_th", "r"), (P.f_stop, "f_stop", "darkred"), (P.f_warn, "f_warn", "orange")):
        ax[0].axhline(v, color=col, lw=.8, ls=":", label=lab)
    if P.f_des > 0:
        ax[0].axhline(P.f_des, color="g", lw=.8, label="f_des")
    ax[0].set_ylabel("N"); ax[0].legend(ncol=3, fontsize=7); ax[0].set_title(f"Scenario {name}")
    ax[1].plot(t, c("a_vib"), label="a_vib (vibration)"); ax[1].plot(t, c("a_trig"), label="a_trig (trigger)")
    ax[1].fill_between(t, 0, c("saturated"), color="r", alpha=.2, label="saturation active")
    ax[1].set_ylabel("cue 0..1"); ax[1].legend(ncol=3, fontsize=7)
    ax[2].plot(t, c("p_h_z") * 1e3, label="p_h (operator ref)"); ax[2].plot(t, c("p_d_z") * 1e3, label="p_d (after saturation)")
    ax[2].plot(t, c("p_c_z") * 1e3, label="p_c (robot cmd)"); ax[2].plot(t, c("tip_z") * 1e3, "k", lw=.8, label="tip (measured)")
    ax[2].set_ylabel("z [mm]"); ax[2].legend(ncol=4, fontsize=7)
    ax[3].plot(t, c("dist_to_surface") * 1e3); ax[3].axhline(0, color="k", lw=.5)
    ax[3].set_ylabel("tip - surface [mm]\n(<0 penetration, >0 lost)")
    ax[4].plot(t, c("n_used_err_deg"), label="normal used vs true"); ax[4].plot(t, c("n_est_err_deg"), ":", label="online estimate vs true")
    ax[4].set_ylabel("deg"); ax[4].set_xlabel("time [s]"); ax[4].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def comparison(results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for name, rows, P in results:
        t = np.array([r["t"] for r in rows])
        ax[0].plot(t, [r["fn"] for r in rows], label=name)
        ax[1].plot(t, [r["a_vib"] for r in rows], label=name)
    ax[0].axhline(results[0][2].f_th, color="r", ls=":", lw=.8)
    ax[0].set_ylabel("normal force [N]"); ax[1].set_ylabel("vibration cue"); ax[1].set_xlabel("time [s]")
    ax[0].legend(ncol=4, fontsize=8); fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def write_csv(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="C", help=f"{list(SCENARIOS)} | planar | curved | all")
    ap.add_argument("--operator", default="reactive", choices=["scripted", "reactive", "dualsense"])
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--clutch-button", type=int, default=None, help="DualSense button held to move (default: always on)")
    args = ap.parse_args()
    names = {"all": PLANAR + CURVED, "planar": PLANAR, "curved": CURVED}.get(args.scenario, [args.scenario])
    outdir = args.outdir or os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir, exist_ok=True)
    results, summ = [], []
    for name in names:
        rows, P = run_scenario(name, args)
        if not rows:
            continue
        write_csv(rows, os.path.join(outdir, f"{name}.csv"))
        dashboard(name, rows, P, os.path.join(outdir, f"{name}.png"))
        m = metrics(name, rows, P)
        summ.append(m)
        results.append((name, rows, P))
        print(f"{name:9s} success={m['success']!s:5s} peak={m.get('fn_peak')}N  t>fth={m.get('t_above_fth_s')}s  "
              f"loss={m.get('contact_loss_max_s')}s  cue_hf={m.get('cue_hf_rms')}  {m.get('fail_reason', '')}")
    if summ:
        keys = list(dict.fromkeys(k for m in summ for k in m))
        with open(os.path.join(outdir, "summary.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(summ)
        with open(os.path.join(outdir, "summary.json"), "w") as f:
            json.dump(summ, f, indent=1)
    if len(results) > 1:
        comparison(results, os.path.join(outdir, "comparison.png"))
    print("saved to", outdir)


if __name__ == "__main__":
    main()
