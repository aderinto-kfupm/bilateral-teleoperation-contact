# Setup guide: from clone to a running simulation

Goal: a PyBullet window with a Franka Panda sliding a probe along a surface, in about 15 minutes. Written for Windows; Linux/macOS notes are marked.

## 1. Get the code
```bash
git clone <repo-url>
cd bilateral-teleoperation-for-surface-contact-tasks
```

## 2. Install Miniconda (once)
Download **Miniconda** from <https://www.anaconda.com/download/success> and install with the defaults.
Find its folder (usually `C:\Users\<you>\miniconda3`); below it is written `<CONDA>`.
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
- Skip `conda init` / `conda activate`; everything below calls the environment's Python directly.

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
| Scripted (default) | `--operator scripted` | nothing |
| Reactive (eases off as the cue grows) | `--operator reactive` | nothing |
| Keyboard | `--operator keyboard --gui` | arrows = x/y, W/S = z |
| Game controller | `--operator dualsense --gui` | PS5 DualSense or Xbox-style pad, connected by USB or dongle (not plain Bluetooth). Pad setup: [../src/haptic-control/QUICKSTART.md](../src/haptic-control/QUICKSTART.md) |

Example: `scripts\sim.cmd --scenario C --operator keyboard --gui`

## 6. Troubleshooting
| Symptom | Fix |
|---|---|
| `franka-haptics env not found` | Redo step 3, or point the launcher at any Python that has the packages: `set SIM_PY=C:\path\to\python.exe` (PowerShell: `$env:SIM_PY="..."`). |
| `missing packages` | Run the `pip install` line printed by the launcher. |
| `self-check FAILED` | Don't run the sim; open an issue with the output. The control law no longer matches the spec. |
| `ModuleNotFoundError: teleop` (manual runs) | Set `PYTHONPATH=src\python`, or `pip install -e src/python`. The launchers do this for you. |
| No GUI window / black window | Update the graphics driver; try without `--gui` first to confirm the physics runs. |
| Numpy/opencv import errors | You upgraded numpy; reinstall `numpy==1.26.0`. |
| Controller not detected | Use USB or a dongle, then re-check with `src/haptic-control/controller_test_ps5.py`. |

## 7. Developer extras
```bash
make env     # .venv with pytest + ruff
make test    # controller self-checks
make lint
```
