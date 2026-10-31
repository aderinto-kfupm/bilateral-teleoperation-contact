# Setup guide: from clone to a running simulation

Goal: a PyBullet window with a Franka Panda sliding a probe along a surface, in about 15 minutes. Written for Windows; Linux/macOS notes are marked.

## 1. Get the code
```bash
git clone https://github.com/aderinto-kfupm/bilateral-teleoperation-contact.git
cd bilateral-teleoperation-contact
```

## 2. Install Miniconda or Anaconda (once)
Download **Miniconda** from <https://www.anaconda.com/download/success> and install with the defaults
(an existing Anaconda install works too). Find its folder (usually `C:\Users\<you>\miniconda3` or
`C:\Users\<you>\anaconda3`); below it is written `<CONDA>`.
```powershell
Get-Command conda | Select-Object Source
```

## 3. Create the `franka-haptics` environment (once)
Run these one at a time in PowerShell. The environment name matters: the launchers look for it.
```powershell
& "<CONDA>\Scripts\conda.exe" create -y -n franka-haptics python=3.12 pip
& "<CONDA>\Scripts\conda.exe" install -y -n franka-haptics -c conda-forge "pybullet=3.25"
& "<CONDA>\envs\franka-haptics\python.exe" -m pip install "numpy==1.26.0" "matplotlib==3.9.2" "opencv-python==4.10.0.82" "pillow==11.3.0" pygame
```
- Keep `numpy==1.26.0`; newer numpy breaks opencv/pybullet in this environment.
- `pygame` is only needed for the game controller; `opencv`/`pillow` only for `src/haptic-control/`.
- Simplest: open an **Anaconda Prompt**, run `conda activate franka-haptics`, and run everything below from
  there. The launchers use the active environment. Without activating, they look for the environment under
  `%USERPROFILE%\miniconda3`, `%USERPROFILE%\anaconda3` and `%LOCALAPPDATA%\miniconda3`.

Check it:
```powershell
& "<CONDA>\envs\franka-haptics\python.exe" -c "import numpy, pybullet, matplotlib; print('OK')"
```
It should print `OK` (plus a pybullet build line).

> Linux/macOS: any Python 3.10+ venv works: `python3 -m venv .venv && .venv/bin/pip install -e "src/python[dev]"`, then `export SIM_PY=$PWD/.venv/bin/python`.

## 4. Run it
From the repo root:

| Shell | Command |
|---|---|
| PowerShell / cmd | `scripts\sim.cmd` |
| Git Bash / Linux / macOS | `./scripts/sim.sh` (or `make sim`) |

The launcher finds the environment, checks the packages, runs the controller self-check (about 1 s), then opens the GUI on **scenario C** (the paper's method). Expected self-check output:
```
ok   test_unsaturated_force_follows_reference_depth
ok   test_saturation_bounds_force_and_virtual_feedback_keeps_growing
ok   test_force_tracking_reaches_desired_force
ok   test_no_phantom_cue_in_free_space
```
Anything after the launcher name is passed to `python -m teleop.run`:
```powershell
scripts\sim.cmd --scenario all --operator scripted    # every scenario, headless, ~1 s each
scripts\sim.cmd --scenario C --gui                    # watch one scenario in real time
```
Headless runs write `results/<timestamp>/` (CSV traces, PNG dashboards, `summary.json`). See [simulation.md](simulation.md) for scenarios and metrics.

## 5. Drive it yourself (optional)
| Operator | Flag | Needs |
|---|---|---|
| Scripted (default for `sim`) | `--operator scripted` | nothing |
| Reactive (eases off as the cue grows) | `--operator reactive` | nothing |
| Keyboard | `--operator keyboard --gui` | click the PyBullet window first |
| Game controller | `--operator dualsense --gui` | PS5 DualSense or Xbox-style pad by USB or dongle. Pad setup: [../src/haptic-control/QUICKSTART.md](../src/haptic-control/QUICKSTART.md) |

Controls:

| Action | DualSense | Keyboard |
|---|---|---|
| Move x / y | left stick | left-right / up-down arrows |
| Move z (into / away from the surface) | right stick, vertical | Page Down / Page Up (hold), or S / W (2 mm per press; hold to repeat) |
| Reset the trial (same scenario, initial state) | Options | R |
| End the trial and save the log | Circle | Q |
| Clutch (optional) | button chosen with `--clutch-button N` (hold to move) | -- |

A trial ends at the end of the yellow line, when you end it, or after 25 s (`--timeout N` to change it for
practice; the terminal prints the time left). The GUI shows the path, the force arrow and the operator (green)
and robot (blue) markers; add `--hud` for the text readout of force and cues.

Feedback: the two vibration motors play the vibration cue (low) and the trigger cue (high). Adaptive-trigger
resistance is not driven (Pygame cannot), it is shown in the overlay only. The baseline gives no cue.

Example: `scripts\sim.cmd --scenario C --operator dualsense --gui`

## 6. Troubleshooting
| Symptom | Fix |
|---|---|
| `franka-haptics env not found` | Run `conda activate franka-haptics` first, redo step 3, or point the launcher at any Python that has the packages: `set SIM_PY=C:\path\to\python.exe` (PowerShell: `$env:SIM_PY="..."`). |
| `missing packages` | Run the `pip install` line printed by the launcher. |
| `self-check FAILED` | Don't run the sim; open an issue with the output. The control law no longer matches the spec. |
| `ModuleNotFoundError: teleop` (manual runs) | Set `PYTHONPATH=src\python`, or `pip install -e src/python`. The launchers do this for you. |
| No GUI window / black window | Update the graphics driver; try without `--gui` first to confirm the physics runs. |
| Numpy/opencv import errors | You upgraded numpy; reinstall `numpy==1.26.0`. |
| Controller not detected (`No controllers found`) | Check that Windows sees it: Win+R, `joy.cpl`. Use a data USB cable (not charge-only) or pair by Bluetooth (hold Create + PS). Close Steam, DS4Windows and DSX (also from the tray): they can grab the pad. Re-check with `python src\haptic-control\controller_test_ps5.py --list`. Meanwhile use `--operator keyboard`. |
| Keyboard does nothing | Click inside the PyBullet window so it has focus. |
| Window closes by itself | The 25 s trial limit was reached (the terminal says so). Use `--timeout 60` to practise. |

## 7. Developer extras
```bash
make env     # .venv with pytest + ruff
make test    # controller self-checks
make lint
```
