# Simulation: safe bilateral teleoperation of surface contact (PyBullet)

Implements the report's architecture (Fig. 1) on a Franka Panda in PyBullet and measures the report's hypotheses H1-H4 (Sec. VII).

```
src/python/teleop/
  controller.py       the paper's control law (pure numpy): blocks 1-4, 8, 9 of Fig. 1
  scene.py            PyBullet: Franka + probe + rigid plane/curved surface, contact wrench, IK (blocks 5-7)
  run.py              scenarios, operators, per-step logging, metrics, plots, GUI overlay
  tests/test_controller.py  4 self-checks of the control law against an ideal spring wall (no PyBullet)
```

## Quick start

```powershell
.\scripts\sim.cmd                             # self-check, then GUI run of scenario C
.\scripts\sim.cmd --scenario all --operator scripted  # any run.py arguments pass straight through
```
`scripts/sim.cmd` finds the `franka-haptics` env itself (override with `SIM_PY`), checks the packages, and runs the self-check first.
Git Bash / Linux / macOS: `./scripts/sim.sh` (or `make sim`) does the same with the same arguments (falls back to `python3` on PATH if the conda env is absent).

## Run (manual)

Use the `franka-haptics` conda env from `src/haptic-control/QUICKSTART.md` (plus `matplotlib==3.9.2`; keep `numpy==1.26.0`, newer numpy breaks opencv/pybullet there).

```powershell
$PY = "$env:USERPROFILE\miniconda3\envs\franka-haptics\python.exe"
$env:PYTHONPATH = "src\python"
& $PY tests	est_controller.py                                  # sanity check, 1 s
& $PY -m teleop.run --scenario all --operator scripted           # H2 test: operator keeps pushing into the surface
& $PY -m teleop.run --scenario all --operator reactive           # H1 test: operator eases off as the cue grows
& $PY -m teleop.run --scenario C --gui                           # watch one scenario in real time
& $PY -m teleop.run --scenario C --operator dualsense --gui      # drive it yourself with the PS5 controller
```
`--scenario` takes `baseline A B C X Xfixed Xoracle`, or `planar` / `curved` / `all`. Each run takes about 1 s of wall time headless.

## Scenarios (report Sec. VII-B)

| Name | Surface | Admittance | Feedback to operator | Saturation | Notes |
|---|---|---|---|---|---|
| `baseline` | plane | off (direct IK) | none (visual only) | no | direct Cartesian teleop |
| `A` | plane | on | **measured** force -> vibration | no | paper scenario A |
| `B` | plane | on | **virtualized** (motion error) | no | paper scenario B |
| `C` | plane | on | virtualized | **yes** | the paper's method |
| `X` | curved (R = 0.3 m) | on | virtualized | yes | extension: `f_des` = 5 N + online normal estimate |
| `Xfixed` | curved | on | virtualized | yes | extension ablation: normal fixed to +z |
| `Xoracle` | curved | on | virtualized | yes | upper bound: ground-truth normal |

## What you can observe

**GUI** (`--gui`): yellow path line; red force arrow at the probe (length and brightness scale with force); green marker = operator reference `p_h`, blue marker = robot command `p_c`; text line with scenario, phase, normal force, `a_vib`, `a_trig`, saturation state.

**Per run, in `results/<stamp>/`:**

| File | Content |
|---|---|
| `<scen>.csv` | every 1/240 s step: operator command `u`; `p_h`, `p_d`, `p_c`; measured tip pose; raw contact force; filtered normal force `fn`; virtual feedback `f_hn`; cues `a_vib`, `a_trig`; saturation flag; contact flag and #points; normal error (used and estimated vs ground truth); tip-to-surface distance; IK error; joint-limit flag; control-step time |
| `<scen>.png` | dashboard: forces vs thresholds, cues + saturation shading, z references vs tip, penetration/contact loss, normal error |
| `summary.csv/.json` | Sec. VII-C metrics: success + failure reason, completion time, peak/mean/std force, RMS force error vs `f_des`, time above `f_th`/`f_stop`, contact-loss count/duration, path error, coverage, cue variation (H1), command oscillation (H1), saturation activity, normal-estimate error, loop timing |
| `comparison.png` | all scenarios overlaid (force and vibration cue) |

Failure = peak normal force > 20 N, contact loss > 1 s, joint-limit hit, or timeout (constants at the top of `teleop/run.py`).

## Results of the shipped configuration (seed 0)

| H | Evidence | Result |
|---|---|---|
| **H2** saturation limits force | scripted operator, peak force: baseline **154 N**, A/B **35.5 N**, C **10.6 N**. Time above `f_stop`: 6.5 / 5.8 / 5.8 / **0 s** | supported |
| **Paper claim**: feedback shows intent, force stays bounded | in C the virtual feedback climbs to about 42 N while the real force stays at about 10 N (`C.png`, top panel) | reproduced |
| **H1** virtualized feedback is smoother | cue variation `cue_hf_rms`: A 1.27 -> B 0.61 (scripted); 1.23 -> 0.52 (reactive) | supported (cue signal). Operator-command oscillation is equal (0.01) because the operator model is simple |
| **H4** normal estimation helps on curved surfaces | RMS force error vs 5 N: `X` **0.28 N**, `Xfixed` 1.30 N, `Xoracle` 0.39 N. Mean normal error 1.8 deg vs 9.4 deg (fixed) | supported |
| **H3** haptics vs visual-only | not testable with a model operator; needs the DualSense with a human (`--operator dualsense`, baseline vs A-C) | open |
| Real-time margin | max control step 0.9 ms against a 4.17 ms period | ok |

## Decisions and deviations you should know about

- **Soft contact is deliberate.** Surface and probe get `contactStiffness`, giving a clean linear spring: effective `Ke` = 5000 N/m (half the per-body 1e4, Bullet combines the two in series). This is what makes the paper's `K_eq` formula (Eq. 9) usable. `Params.Ke` must match; if you change `Scene(Ke=...)`, set `Params.Ke = Ke/2`.
- **Rest position estimate.** The paper takes `p_e` as the last compliant command when saturation starts. That command already sits `fn/Ke` inside the spring, so the force settled at 12.4 N instead of 10 N. `Params.rest_correction=True` (default) adds `fn/Ke`; set it to `False` for the paper-literal behaviour.
- **Saturation release.** The paper does not say when saturation ends. Here it releases once the force is under half of `f_th` and the operator reference is no deeper than the saturation value.
- **Extension force tracking** uses the admittance plus an outer integral loop on the normal force error (`k_force`). The admittance alone only reaches `f_des/KP` past its reference and lost contact on the falling dome (found in testing).
- **Phantom cue removed.** With `f_des` > 0 the admittance holds a `f_des/KP` offset in free space, which made the virtual cue buzz before contact; it is subtracted. In the extension the cue is therefore a deviation-from-desired-force cue.
- **Frames.** Everything is in the world-aligned base frame with fixed tool orientation, so the rotation matrices in Eqs. (4)-(7) are identity. Saturation acts along the surface normal, which equals the paper's per-axis rule on the plane.
- **DualSense.** Sticks drive the probe; vibration motors play `a_vib` (low) and a scaled `a_trig` (high). Adaptive-trigger resistance needs `pydualsense`/HID and is **not** driven; it is logged and shown in the overlay only.
- **Operators are models, not people.** `scripted` pushes into the surface on purpose (tests the safety mechanism); `reactive` adds a 150 ms delay and a lag and eases off as the cue grows. Neither replaces the user study.
- Not modelled: communication delay (the report excludes it; control step time is logged), orientation control, force-sensor noise beyond a 1st-order filter.
