# Bilateral Teleoperation for Safe Surface-Contact Tasks

LaTeX version of the SCE571 *Robot Dynamics and Control* Phase 2 report.

- `report.tex` is the source (IEEEtran). Build it with `pdflatex report.tex` (run twice for references).
- `report.pdf` is the compiled output (6 pages, same as the original PDF).

## What is this paper about?

**The big idea:** a person remotely guides a robot arm to slide a probe along a surface (inspecting it, writing on a board, sanding), and *feels* what the robot feels. This is called **bilateral teleoperation**: commands go to the robot, and touch comes back to the human.

The project studies and re-creates a 2024 method by Pagliara et al., "Safe haptic teleoperations of admittance controlled robots with virtualization of the force feedback". The report is the Phase 2 plan: understand the paper, adapt it to hardware we have, review related work, and plan the experiments.

## The problem, in plain words

Imagine pushing a pen against a wall through a long robotic arm. Two things go wrong:

1. **Jerky feel.** If the robot simply sends back the raw force sensor reading, the sensation jumps when the pen touches or leaves the wall. The operator's hand gets shoved, loses accuracy, and the system can start to shake.
2. **Runaway force.** A "soft" (compliant) robot does not stop you from pushing harder. If your hand keeps pushing into the wall, the force keeps growing until something breaks.

## The three ingredients of the solution

| Ingredient | Everyday analogy | What it fixes |
|---|---|---|
| **Admittance control** | The robot acts like it is on a soft spring: it gives way when it meets resistance. | Gentle, compliant contact. |
| **Virtualized force feedback** | Instead of feeling the harsh sensor reading, you feel a *smooth, simulated spring* built from how far the robot's actual position lags behind where you told it to go. | Jerky feedback. |
| **Reference saturation** | A safety "stop": once the force hits a limit, the robot stops following your hand deeper into the surface. You still *feel* yourself pushing, but the robot doesn't push harder. | Runaway force. |

The original paper tested this with a robot writing A-C-G on a blackboard. Three setups were compared (A: raw force feedback, B: smooth feedback, C: smooth feedback plus the safety stop). Only C wrote correctly on the first try, and its peak force was far closer to a human's (Table I of the report). The authors note this comes from a small number of trials, not a full user study.

## What our project does differently

The original used a special force-feedback stylus and a real UR10 robot. We don't have those, so we adapt it:

| Original paper | Our version |
|---|---|
| 3D Systems Touch haptic stylus | **PS5 DualSense controller** |
| Real UR10 robot with force sensor | **Franka arm simulated in PyBullet** |
| Real force pushed back into the hand | **Vibration and adaptive-trigger resistance** (touch "translated" into buzzes and a stiffening trigger) |

The DualSense cannot push your hand with a real force, so this is a **sensory-substitution adaptation, not an exact copy**. The report is careful to say so.

## Goals

1. Reproduce the paper's core control method on a flat surface in simulation.
2. Check whether its claims hold with our adapted hardware (see the hypotheses below).
3. **Extension (our contribution):** work on **curved surfaces** by tracking a desired normal force and estimating the surface direction online while moving.

## What the report contains

- **Sections I-III:** the problem, the original system, and the controller equations (admittance, force limit, virtual feedback, DualSense mapping).
- **Sections IV-V:** which signals are needed and how each is obtained in PyBullet, plus the block diagram.
- **Section VI:** literature review of 10 supporting papers (stability, haptics, surface treatment, force control).
- **Section VII:** experiment plan with five conditions, metrics and hypotheses.

### Hypotheses to be tested

- **H1:** smooth (virtualized) feedback is less jittery than direct force feedback.
- **H2:** the safety stop (saturation) lowers peak force and failures.
- **H3:** tactile feedback beats visual-only operation without making the task much slower.
- **H4 (extension):** estimating the surface normal improves force and path accuracy on curved surfaces.

## Phases

Phase 2 (this report) is the plan. Phase 3 builds the simulated environment and controller, and Phase 4 compares Scenarios A-C.
