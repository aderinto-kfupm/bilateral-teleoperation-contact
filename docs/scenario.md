# Scenario specification (Phase 3)

Surface inspection with a teleoperated probe: the operator slides a probe along a rigid surface while the
normal contact force must stay positive (no contact loss) and below a damage limit. Values below are the
constants in `src/python/teleop/scene.py`, `run.py` and `controller.py`.

![Scenario C after 4 s of a scripted run](../figures/sim_scene.png)

*Scenario C, 4 s into a scripted run: the probe (red) slides along the plate at 10 N. Regenerate with
`python -m teleop.render`.*

## Objects and workspace

| Item | Value |
|---|---|
| Robot | Franka Panda (pybullet_data URDF), fixed base on a table, joint position control through IK |
| Tool | rigid sphere, radius 10 mm, fixed 130 mm along the hand z-axis; tool orientation held constant |
| Surface (planar scenarios) | box, top face 120 mm above the table, 0.50 m x 0.40 m |
| Surface (extension) | convex cylinder, radius 300 mm, axis along y |
| Contact | effective stiffness 5000 N/m (Bullet combines probe and surface springs in series), Coulomb friction 0.3 |
| Simulation | 240 Hz, 100 solver iterations |
| Constraints | joint limits (failure within 0.02 rad); collisions disabled between probe and arm |

## Initial conditions

The arm starts at `Q_HOME = [0, -0.5, 0, -2.4, 0, 1.9, 0.785]` rad, then the probe is servoed for 1 s to
30 mm above the surface at x = 0.45 m, y = 0. The controller state (admittance, filters, saturation latch)
is reset to zero at that point.

## Task sequence

1. **Approach**: lower the probe until contact (filtered normal force > 0.5 N).
2. **Trace**: move along +x from 0.45 m to 0.65 m at y = 0 while keeping contact.
3. **End**: the run stops 0.5 s after the probe reaches x = 0.65 m, or when the operator ends it.

## Success and failure

A run **succeeds** if the end of the path is reached with the path covered in contact, and none of the
failures below occurs.

| Failure | Condition |
|---|---|
| No contact | the probe never touches the surface |
| Low coverage | fewer than 18 of the 20 path segments (90 %) touched in contact |
| Ended early | the operator ends the trial (Circle / Q) before reaching x = 0.65 m |
| Excessive force | filtered normal force > 20 N at any time during the trace |
| Contact loss | contact lost for more than 1 s continuously after first contact |
| Joint limit | any joint within 0.02 rad of its limit |
| Timeout | path end not reached within 25 s |

Logged metrics (same for every scenario, used again in Phase 4): completion time, peak/mean/std normal
force, time above `f_th` and `f_stop`, contact-loss count and duration, path error, coverage (20 path
segments touched in contact), cue variation, command oscillation, saturation activity, normal-estimate error,
loop timing. See `docs/simulation.md`.

## Reset procedure

* Every run builds a fresh PyBullet world, so each scenario starts from the same state.
* Interactive runs: **Options** on the DualSense or **R** on the keyboard restarts the current trial from
  the initial conditions above; **Circle** or **Q** ends it and saves the log.
* Headless runs are deterministic: the same command gives identical traces.

## Baseline

`--scenario baseline`: direct Cartesian teleoperation. The operator's reference goes straight to IK and
joint position control, with no admittance, no saturation and no haptic cue (visual feedback only).

## Pilot experiments: neither trivial nor impossible

`python -m teleop.pilot` sweeps how far the scripted operator pushes its reference past the surface
(results/pilot_<stamp>/, figure `figures/pilot.png`):

| Push depth (mm) | 0 | 1 | 2 | 3 | 4 | 5 | 7.5 | 10 | 20 | 30 |
|---|---|---|---|---|---|---|---|---|---|---|
| baseline peak force (N) | 8.3 | 8.3 | 12.7 | 18.7 | **23.9** | **28.9** | **41.5** | **54.0** | **104** | **154** |
| baseline pass | yes | yes | yes | yes | no | no | no | no | no | no |
| C peak force (N) | 5.6 | 5.6 | 5.6 | 5.6 | 5.6 | 6.7 | 9.6 | 10.6 | 10.6 | 10.6 |
| C pass | yes | yes | yes | yes | yes | yes | yes | yes | yes | yes |

* **Not impossible**: the baseline passes when the operator holds the reference within about 3 mm of the
  surface, and the paper's method (C) passes at every depth.
* **Not trivial**: with visual feedback only, a 4 mm overshoot of the reference already breaks the 20 N limit,
  and an operator who keeps pushing (30 mm) reaches 154 N. A cue-reactive operator still fails the baseline
  (no cue) but passes A, B and C at 5.6 N.
