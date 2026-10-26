# Franka Panda Controlled Simulation Guide

**This guide explains how to install, test, and run the Franka Panda tabletop simulation with a PS5 DualSense controller.**

The final controlled demo supports:

- A Franka Panda arm mounted on the table.
- Controller-based Cartesian end-effector movement.
- Controller buttons for opening and closing the gripper.
- Colored cubes spawned on the tabletop.
- Rumble feedback when the hand or fingers contact a cube.
- Longer rumble feedback when both fingers contact the same cube.
- Optional overhead and wrist cameras.
- RGB, depth, and segmentation data for vision experiments.

You need:

- A PS5 DualSense controller for the controlled demo.
- The controller connected by USB or Bluetooth and charged.

The simulation can run without a controller, but the `--controller` option requires a connected DualSense.

## Table of Contents

1. [Installation](#1-installation)
   - [Platform support](#11-platform-support)
   - [Install Miniconda](#12-install-miniconda)
   - [Initialize Conda on Windows](#13-initialize-conda-on-windows)
   - [Initialize Conda on macOS](#14-initialize-conda-on-macos)
   - [Initialize Conda on Linux](#15-initialize-conda-on-linux)
   - [Create the franka-haptics environment](#16-create-the-franka-haptics-environment)
   - [Verify the active Python environment](#17-verify-the-active-python-environment)
   - [Install dependencies](#18-install-dependencies)
   - [Verify the installation](#19-verify-the-installation)
2. [File Overview](#2-file-overview)
3. [Demo Videos](#3-demo-videos)
4. [Controller Setup and Testing](#4-controller-setup-and-testing)
   - [Connect and configure the PS5 DualSense](#41-connect-and-configure-the-ps5-dualsense)
   - [Test the controller before the robot](#42-test-the-controller-before-the-robot)
5. [PS5 DualSense Control Mapping](#5-ps5-dualsense-control-mapping)
6. [Run the Final Controlled Demo](#6-run-the-final-controlled-demo)
7. [How Movement and Feedback Work](#7-how-movement-and-feedback-work)
8. [Speed and Control Tuning](#8-speed-and-control-tuning)
9. [Rumble Intensity and Timing](#9-rumble-intensity-and-timing)
10. [Troubleshooting](#10-troubleshooting)
11. [Quick Command Reference](#11-quick-command-reference)

---

## 1. Installation

> **Target Python:** 3.12.x. This project standardizes on **Miniconda** as the only recommended Python/environment manager.
>
> ```
> Miniconda
>     +-- franka-haptics
>           +-- Python 3.12
>           +-- PyBullet (conda-forge)
>           +-- NumPy
>           +-- OpenCV
>           +-- Pillow
>           +-- Pygame
> ```
>
> Install Miniconda once, create the `franka-haptics` environment, activate it when working on
> the project, and deactivate it afterward. Miniconda is the only tested and officially supported
> path for this project. You may still use alternative managers such as `pyenv`, full Anaconda,
> system-wide Python, or `python -m venv` at your own risk, but problems arising from those
> setups will not be addressed by the TAs.

The normal setup path is:

```
Install Miniconda -> Initialize Conda -> Create franka-haptics -> Activate franka-haptics
        -> Install dependencies -> Connect/test DualSense -> Run the simulation
```

### 1.1 Platform support

> **Windows and macOS installations have been tested and verified.** Linux has not yet undergone the same level of validation, so Linux users may need to adapt some setup or controller-related steps themselves. Linux is not unsupported -- the software is expected to be portable -- but installation, graphics, SDL, USB, and controller-driver behavior on Linux has not been comprehensively tested and may require platform-specific troubleshooting.

### 1.2 Install Miniconda

Download **Miniconda** (not the full Anaconda distribution) from the official page:

<https://www.anaconda.com/download/success?reg=skipped>

> **Use Miniconda, not Anaconda.** Miniconda provides Conda and the Python environment-management functionality required by this project without installing the much larger collection of packages included with the full Anaconda distribution.

Run the graphical installer and review the license terms to decide whether to accept them. If you accept, we recommend using the default settings (per-user installation). You do not need to install the full Anaconda distribution for this project.

> **Already have Conda?** If `conda --version` already works on your system, you can skip to [Section 1.6 Create the franka-haptics environment](#16-create-the-franka-haptics-environment).

### 1.3 Initialize Conda on Windows

#### 1.3.1 Why `conda` is not recognized

After installing Miniconda with the standard per-user installer, opening a new PowerShell window may still produce:

```text
conda : The term 'conda' is not recognized as the name of a cmdlet, function, script file, or operable program.
```

This does not necessarily mean Miniconda failed to install -- it usually means the shell has not been initialized for Conda yet.

#### 1.3.2 Initialize PowerShell

With the standard per-user Miniconda installation, initialize PowerShell using:

```powershell
& "$env:LOCALAPPDATA\miniconda3\Scripts\conda.exe" init powershell
```

Then disable automatic activation of the Conda `base` environment:

```powershell
& "$env:LOCALAPPDATA\miniconda3\Scripts\conda.exe" config --set auto_activate_base false
```

What these commands do:

- `conda init powershell` adds Conda shell initialization to your PowerShell profile so that `conda` is available in new PowerShell sessions.
- `conda config --set auto_activate_base false` prevents the `base` environment from activating every time PowerShell starts; a project environment is activated only when you run `conda activate` explicitly.

This gives the intended behavior:

- `conda` is available from PowerShell.
- The `base` environment does not activate automatically.
- The project environment is activated only when explicitly requested.

#### 1.3.3 Verify the shell

Close **all** PowerShell and Windows Terminal sessions and open a new PowerShell terminal, then verify:

```powershell
conda --version
conda info --base
```

If troubleshooting is needed, also check:

```powershell
Get-Command conda
where.exe conda
```

`$env:LOCALAPPDATA\miniconda3` (`%LOCALAPPDATA%\miniconda3`) is the common default per-user installation location. If you selected a different installation directory, adjust the path accordingly. Do not add the entire Miniconda installation manually to the global Windows `PATH` unless required for troubleshooting --- prefer normal Conda shell initialization.

### 1.4 Initialize Conda on macOS

Download the appropriate Miniconda installer for your architecture (Apple Silicon / ARM64 or Intel / x86-64) from the official page above and run the installer.

For the normal `zsh` shell (default on recent macOS), initialize Conda with:

```bash
conda init zsh
conda config --set auto_activate_base false
```

Then restart Terminal or run:

```bash
source ~/.zshrc
```

For users explicitly using Bash:

```bash
conda init bash
conda config --set auto_activate_base false
```

Restart the shell after initialization, then verify:

```bash
conda --version
conda info --base
```

### 1.5 Initialize Conda on Linux

> Linux has not been fully validated by the project maintainers. The steps below are the standard Miniconda flow and are expected to be portable, but Linux users may need to adapt dependency, graphics, SDL, USB, or controller-driver steps where necessary.

Download and run the official Miniconda Linux installer appropriate to your architecture, then initialize your shell. For Bash:

```bash
conda init bash
conda config --set auto_activate_base false
```

Restart the shell and verify with `conda --version` and `conda info --base`.

### 1.6 Create the franka-haptics environment

Create exactly one project environment named `franka-haptics`:

```bash
conda create -n franka-haptics python=3.12 pip
```

Follow the on-screen instructions to review and confirm the creation.

Activate it:

```bash
conda activate franka-haptics
```

A successful activation should produce a prompt similar to:

```text
(franka-haptics)
```

For example on Windows:

```text
(franka-haptics) PS C:\Users\username\...
```

All installation, controller-test, and simulation commands in this guide assume that `franka-haptics` is active.

To leave the environment when you are done:

```bash
conda deactivate
```

### 1.7 Verify the active Python environment

Before installing dependencies, verify that the active interpreter belongs to `franka-haptics`:

```bash
python --version
python -m pip --version
```

The Python version should be `3.12.x`.

On Windows also check:

```powershell
where.exe python
where.exe pip
```

When `franka-haptics` is active, the selected Python should resolve inside the Conda environment rather than to a system Python or an old installation.

You can also list environments:

```bash
conda info --envs
```

The `*` should appear beside `franka-haptics`.

### 1.8 Install dependencies

#### 1.8.1 How Conda and pip are used

This project uses Conda and pip deliberately:

- **Conda** manages the Python interpreter and the environment itself.
- **conda-forge** is preferred for dependencies that require compiled, platform-specific binaries (notably PyBullet).
- **pip** is appropriate for the remaining Python packages.
- **`python -m pip`** is preferred over bare `pip` because it unambiguously uses pip belonging to the currently active Python interpreter. Do not install the same dependency with both Conda and pip.

#### 1.8.2 Install PyBullet via conda-forge

PyBullet requires special treatment on Windows with Python 3.12: older PyBullet releases may not provide a suitable precompiled Windows/Python 3.12 wheel on PyPI, and `pip` may attempt to compile from source. Install the pinned, tested PyBullet build through `conda-forge`:

```bash
conda activate franka-haptics
conda install -c conda-forge pybullet==3.2.5
```

#### 1.8.3 Install the remaining packages

Then install the remaining required packages with pip:

```bash
python -m pip install "numpy==1.26.0" "opencv-python==4.10.0.82" "pillow==11.3.0" pygame
```

Keep the exact pinned versions shown here -- they are the tested set for this project.

### 1.9 Verify the installation

#### 1.9.1 Check the environment

```bash
python --version
conda info --envs
python -m pip --version
```

#### 1.9.2 Verify imports

Then verify imports:

```bash
python -c "import numpy, pybullet, cv2, PIL, pygame; print('All required packages imported successfully.')"
```

#### 1.9.3 Optionally print versions

Optionally print package versions:

```bash
python -c "import numpy, cv2, PIL, pygame; print('NumPy:', numpy.__version__); print('OpenCV:', cv2.__version__); print('Pillow:', PIL.__version__); print('Pygame:', pygame.__version__)"
```

---


## 2. File Overview

### `controller_test_ps5.py`

Standalone PS5 DualSense diagnostic program. It can list controllers, print live axis/button/D-pad events, read normalized R2 force, test the logical R2 lock, and test rumble. The final controlled bridge imports its reusable controller interface.

Tested as described in [Section 4 Controller Setup and Testing](#4-controller-setup-and-testing) — see [4.1 Connect and configure](#41-connect-and-configure-the-ps5-dualsense), [4.2 Test before the robot](#42-test-the-controller-before-the-robot), and the controller diagnostics in [Section 11 Quick Command Reference](#11-quick-command-reference). Controller-related troubleshooting is in [Section 10 Troubleshooting](#10-troubleshooting).

### `pybullet_vision_env_controlled.py` (internal — not run directly)

Internal controlled-environment module. It implements the table mounting, controller-to-IK movement, gripper commands, and hand/finger contact checks used by the final demo. It is not invoked directly from the command line; `vision_bridge_controlled.py` imports `VisionEnv` from this file (`from pybullet_vision_env_controlled import VisionEnv` at `vision_bridge_controlled.py:12`).

It is exercised indirectly through [Section 6 Run the Final Controlled Demo](#6-run-the-final-controlled-demo) (including safe mode, overhead/wrist cameras, and segmentation) and explained in [Section 7 How Movement and Feedback Work](#7-how-movement-and-feedback-work). Environment-related issues (mounting, collisions, OpenGL) are covered in [Section 10 Troubleshooting](#10-troubleshooting).

### `vision_bridge_controlled.py`

Final controller and vision runner. It imports the controlled environment, reads the DualSense, commands the robot, checks contact/grasp state, triggers rumble, and optionally runs the camera pipeline.

Run and tuned as described in [Section 6 Run the Final Controlled Demo](#6-run-the-final-controlled-demo), [Section 8 Speed and Control Tuning](#8-speed-and-control-tuning), and [Section 9 Rumble Intensity and Timing](#9-rumble-intensity-and-timing). See [Section 11 Quick Command Reference](#11-quick-command-reference) for the canonical demo commands and [Section 10 Troubleshooting](#10-troubleshooting) if the arm does not move or rumble fails.

---

## 3. Demo Videos

> OneDrive / SharePoint links require KFUPM sign-in. Use **Download** in the SharePoint viewer if the video does not play inline. If a link asks for permission, request access from `ramy.rashad@kfupm.edu.sa`.

| # | Demo | Link (OneDrive / SharePoint) | Preview |
|---|---|---|---|
| 1 | Arm controller working -- DualSense driving the Franka Panda with contact rumble | [arm_controller_working.mp4](https://kfupmedusa-my.sharepoint.com/:v:/g/personal/ramy_rashad_kfupm_edu_sa/IQCXPp53oZy2TKSjceuDKeSdAXQIqTMcuwvinx-wxCXVWB8?e=fuer76) | Click the link to play or download. On the SharePoint page use `...` -> **Download** if playback is blocked. |
| 2 | Arm working (screen capture) -- overhead and wrist camera view | [arm_working_screen.mp4](https://kfupmedusa-my.sharepoint.com/:v:/g/personal/ramy_rashad_kfupm_edu_sa/IQDS_nuB4n-DTrfNhFSYLdpeAUo_cREFnUXHQFdLKNYEapU?e=0fraNd) | Click the link to play or download. |
| 3 | Franka grasp preview | [franka.gif](https://kfupmedusa-my.sharepoint.com/:i:/g/personal/ramy_rashad_kfupm_edu_sa/IQCudC8TCo3iQ4S_tbl0A-o0AWuhIIDkQR1TgcRxBgJFpXU?e=VFFNl9) | Click the link to view or download. |

---

## 4. Controller Setup and Testing

### 4.1 Connect and configure the PS5 DualSense

1. Connect the DualSense by USB or pair it through Bluetooth.
2. Turn on the controller and keep it connected while running the simulation.
3. Close other applications that may already be using the controller.

If the controller is detected but rumble does not work, reconnect it by USB and repeat the diagnostics. Different Windows, Bluetooth, and SDL driver combinations can expose different controller capabilities.

> **Note:** On some Windows USB configurations the DualSense may appear as an audio device. If rumble is silent, check that the device volume is not muted and that the correct output device is selected. This is an observed driver behavior that can affect haptic behavior in some setups -- it is not a universal requirement.

### 4.2 Test the controller before the robot

The controller test is independent of PyBullet. It confirms that the operating system, Pygame, axes, buttons, and rumble are working before starting the robot.

All commands in this section assume `franka-haptics` is active (`conda activate franka-haptics`).

#### 4.2.1 List controllers and read axes / D-pad

##### List controllers

```bash
python controller_test_ps5.py --list
```

Expected output resembles:

```text
[0] DualSense Wireless Controller | axes=6 buttons=13 hats=1
```

If no controller is listed, check the USB/Bluetooth connection, controller charge, and Windows game-controller settings.

##### Read axes, buttons, and D-pad

```bash
python controller_test_ps5.py --duration 10
```

Move both sticks, press the buttons, and use the D-pad. Example output:

```text
axis 0: +1.000
button 0: DOWN
button 0: UP
hat 0: (0, 1)
```

Stop a test that runs indefinitely with `Ctrl+C`. When you press a button, it will show `DOWN` then `UP` when released. Small fluctuations in joystick readings near zero are normal.

#### 4.2.2 Test rumble

##### Test startup rumble

```bash
python controller_test_ps5.py --rumble-test --duration 3
```

For a stronger and longer test:

```bash
python controller_test_ps5.py --rumble-test --rumble-strength 1.0 --rumble-duration 1000 --duration 3
```

The strength range is `0.0` to `1.0`. Duration is in milliseconds.

##### Test rumble after a button press

```bash
python controller_test_ps5.py --rumble-on-button --duration 10
```

For both startup rumble and button rumble:

```bash
python controller_test_ps5.py --rumble-test --rumble-on-button --rumble-strength 0.8 --rumble-duration 300 --duration 10
```

Some DualSense driver configurations do not respond to the initial rumble command until the controller has received an input event. If startup rumble does not work, press a button once and test again. This behavior is controller/driver dependent; it does not necessarily mean the Python code failed.

#### 4.2.3 R2 diagnostics

##### R2 force reading

```bash
python controller_test_ps5.py --duration 10 --r2-force-scale 1.5
```

R2 is normalized to a `0.0..1.0` value. The script also prints a scaled force value for use by grip or motor-control code:

```text
R2 axis 5: 0.420 (force=0.630)
```

##### R2 logical lock at 50%

```bash
python controller_test_ps5.py --duration 10 --r2-lock --r2-threshold 0.5 --rumble-strength 1.0
```

When R2 reaches 50%, the script reports `R2 LOCKED` and sends a maximum-strength vibration pulse. When the value drops below 50%, it reports `R2 UNLOCKED` and sends another pulse. This is a software state; it does not physically stop or harden the trigger.

#### 4.2.4 Example output and options

##### Example output while running

Your exact values will vary with the controller and how quickly the trigger is pressed, but a typical session looks like this:

```text
pygame 2.6.1 (SDL 2.32.56, Python 3.12.14)
Using [0] DualSense Wireless Controller
axes=6 buttons=13 hats=1
Move a control or press a button. Press Ctrl+C to stop.

R2 axis 5: 0.420 (force=0.420)
axis 5: +0.420
R2 axis 5: 0.537 (force=0.537)
R2 LOCKED
axis 5: +0.537
R2 axis 5: 1.000 (force=1.000)
axis 5: +1.000
R2 axis 5: 0.459 (force=0.459)
R2 UNLOCKED
axis 5: +0.459
R2 axis 5: 0.000 (force=0.000)
axis 5: -1.000
```

The `R2 axis 5` line is the normalized trigger value. The separate `axis 5` line is the raw SDL axis value, which commonly ranges from `-1.0` to `+1.0` on this controller. The vibration is felt at the `R2 LOCKED` and `R2 UNLOCKED` transitions, but it is not printed as a separate message.

##### DualSense adaptive-trigger note

This project currently uses Pygame/SDL for controller input and rumble. It does not implement DualSense adaptive-trigger resistance. The `R2 LOCKED` state used here is therefore a software threshold with rumble feedback, not a physical trigger stop.

##### Controller test options

| Option | Meaning | Default |
| --- | --- | --- |
| `--list` | List connected controllers and exit | Off |
| `--index N` | Select controller number `N` | `0` |
| `--duration S` | Stop after `S` seconds; `0` runs until `Ctrl+C` | `0` |
| `--poll-interval S` | Delay between event polls | `0.01` |
| `--deadzone V` | Ignore small axis changes | `0.05` |
| `--r2-threshold V` | R2 logical lock threshold | `0.25` (effective minimum `0.5`) |
| `--r2-force-scale V` | Scale the printed R2 force value | `1.0` |
| `--r2-lock` | Enable the logical R2 lock state | Off |
| `--rumble-test` | Vibrate once at startup | Off |
| `--rumble-on-button` | Vibrate after every button press | Off |
| `--rumble-strength V` | Rumble strength from `0.0` to `1.0` | `0.5` |
| `--rumble-duration MS` | Rumble duration in milliseconds | `150` |

---

## 5. PS5 DualSense Control Mapping

The following is the standard mapping reported by `pygame` for a Wireless DualSense in the active environment.

### 5.1 Standard PS5 DualSense inputs

#### 5.1.1 Buttons

| Pygame input | PS5 control |
| --- | --- |
| `button 0` | Cross / X |
| `button 1` | Circle / O |
| `button 2` | Square / □ |
| `button 3` | Triangle / △ |
| `button 4` | L1 |
| `button 5` | R1 |
| `button 6` | L2 digital button |
| `button 7` | R2 digital button |
| `button 8` | Share |
| `button 9` | Options |
| `button 10` | Left stick click |
| `button 11` | Right stick click |
| `button 12` | PS button |

The mapping below matches the DualSense output observed with Pygame in the active environment. Always trust the live output from `controller_test_ps5.py` if a different driver reports different numbers.

#### 5.1.2 D-pad

The D-pad is `hat 0`, with `(x, y)` values:

| Pygame input | PS5 control |
| --- | --- |
| `hat 0: (0, 1)` | Up |
| `hat 0: (0, -1)` | Down |
| `hat 0: (-1, 0)` | Left |
| `hat 0: (1, 0)` | Right |
| `hat 0: (0, 0)` | Released |

#### 5.1.3 Axes

Typical DualSense axis numbering is:

| Axis | PS5 control | Normal range |
| --- | --- | --- |
| `axis 0` | Left stick horizontal | `-1.0` to `+1.0` |
| `axis 1` | Left stick vertical | `-1.0` to `+1.0` |
| `axis 2` | Right stick horizontal | `-1.0` to `+1.0` |
| `axis 3` | Right stick vertical | `-1.0` to `+1.0` |
| `axis 4` | L2 trigger | `0.0` to `1.0` or `-1.0` to `+1.0` depending on driver |
| `axis 5` | R2 trigger | `0.0` to `1.0` or `-1.0` to `+1.0` depending on driver |

In practice, the script normalizes the trigger to a clean `0.0..1.0` force value before using it for lock or force logic.

```text
R2 axis 5: 0.420 (force=0.420)
```

To enable the software lock at 50% and maximum transition vibration:

```bash
python controller_test_ps5.py --duration 10 --r2-lock --r2-threshold 0.5 --rumble-strength 1.0
```

`R2 LOCKED` is printed at or above the threshold and `R2 UNLOCKED` when the value falls below it. This project currently uses Pygame/SDL and does not implement DualSense adaptive-trigger resistance, so this lock is a software state with haptic feedback only.

### 5.2 Robot controls

| Controller input | Robot action |
| --- | --- |
| Left stick horizontal, axis 0 | Move end effector in X |
| Left stick vertical, axis 1 | Move end effector in Y |
| Right stick vertical, axis 3 | Move end effector in Z |
| Button 0, Cross / X | Close gripper |
| Button 1, Circle / O | Open gripper |

The bridge applies a deadzone so small stick drift does not move the robot.

---

## 6. Run the Final Controlled Demo

During the final demo:

1. Move the arm above a cube with the sticks.
2. Press Cross / X to close the gripper.
3. Use the sticks to bring the fingers around the cube.
4. Watch for the short contact rumble.
5. Continue closing or repositioning until both fingers touch the cube.
6. Watch for the longer grasp rumble.
7. Use Circle / O to open the gripper.

The simulation is a control and vision prototype. The grasp detector is based on PyBullet contact links, so it confirms simultaneous finger contact rather than proving that the cube is physically lifted and retained under all dynamics.

### 6.1 Final demonstration (recommended)

#### 6.1.1 Normal demo

**Use this command for the normal final demonstration:**

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.03 --use-overhead --physics-steps 4 --steps 3000
```

The command opens the PyBullet GUI, mounts the robot on the table, enables the DualSense, enables the overhead camera, and stops after `--steps 3000` control cycles. The default `--steps` value is `-1` (run until `Ctrl+C` or the window is closed), so `--steps 3000` is included explicitly where a finite run is desired.

#### 6.1.2 Steps flag note

Stop the demo with `Ctrl+C` or close the simulation window.

### 6.2 Variants

#### 6.2.1 Diagnostic run

Use this when checking controller values or investigating movement:

```bash
python vision_bridge_controlled.py --gui --controller --debug-controller --use-overhead --print-every 30 --steps 3000
```

Every 30 cycles it prints the latest axis values. When a stick is moved, the corresponding axis should become significantly different from zero.

#### 6.2.2 Run without the controller

The controlled bridge can also display the simulation without reading the DualSense:

```bash
python vision_bridge_controlled.py --gui --use-overhead --steps 1000
```

Without `--controller`, the arm is not commanded by the joystick.

#### 6.2.3 Both cameras

```bash
python vision_bridge_controlled.py --gui --controller --use-overhead --use-wrist --steps 3000
```

#### 6.2.4 Save annotated frames

```bash
python vision_bridge_controlled.py --gui --controller --use-overhead --save output --steps 3000
```

Frames are saved in the `output` directory using names such as `overhead_000000.png`.

#### 6.2.5 Segmentation detector

```bash
python vision_bridge_controlled.py --gui --controller --use-overhead --detector seg --steps 3000
```

The segmentation detector groups rendered pixels by PyBullet body/link ID. It is a prototype detector for testing the RGB-D and segmentation pipeline, not a general object-recognition model.

#### 6.2.6 Safe mode

For slower computers or graphics-driver problems:

```bash
python vision_bridge_controlled.py --gui --controller --safe-mode --steps 3000
```

Safe mode uses a 320x240 overhead image and TinyRenderer. It is slower visually but avoids dependence on hardware OpenGL rendering.

---

## 7. How Movement and Feedback Work

The controlled bridge performs the following loop:

1. Read the DualSense axes and button states using Pygame.
2. Convert joystick values into a small Cartesian end-effector target change.
3. Use PyBullet inverse kinematics to convert the target into Panda joint targets.
4. Apply position control to the seven arm joints.
5. Apply position control to the two gripper joints.
6. Step the physics simulation.
7. Check robot-cube contacts.
8. Trigger rumble when a new hand/finger contact or grasp is detected.

The robot is mounted on the tabletop by default at approximately `x=0.15`, while the cubes are spawned near `x=0.45` to `x=0.525`. This places the base away from the main cube workspace and gives the arm room to reach toward the cubes.

Contact feedback is event-based rather than continuous:

- A short rumble occurs when the Panda hand or fingers first contact a cube.
- A longer rumble occurs when both finger links contact the same cube.
- Holding contact does not repeatedly vibrate every simulation frame.

---

## 8. Speed and Control Tuning

The final bridge exposes two main speed-related options plus base placement.

### 8.1 End-effector speed

Default:

```text
--control-speed 0.012
```

This is the Cartesian target change applied per controller update, in meters.

#### 8.1.1 Faster response

Increase it for faster stick response:

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.018 --steps 3000
```

The tested faster setting is:

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.03 --physics-steps 4 --steps 3000
```

A conservative range to try is approximately `0.008` to `0.03`.

#### 8.1.2 Slower / finer control

If the arm becomes difficult to control, reduce it:

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.006 --steps 3000
```

### 8.2 Physics steps per controller update

Default:

```text
--physics-steps 2
```

#### 8.2.1 What it does

This controls how many PyBullet physics steps run after each controller sample.

#### 8.2.2 Tuning

Try this faster setting:

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.018 --physics-steps 3 --steps 3000
```

Higher values advance the simulation farther per controller update. They do not necessarily make the controller more precise. If motion becomes jumpy, return to `--physics-steps 2`.

### 8.3 Robot base position

#### 8.3.1 Move the base

The table-mounted robot X position can be changed at launch:

```bash
python vision_bridge_controlled.py --gui --controller --robot-x 0.10 --steps 3000
```

The default is `0.15`. The Y position can also be changed:

```bash
python vision_bridge_controlled.py --gui --controller --robot-x 0.15 --robot-y 0.10 --steps 3000
```

#### 8.3.2 Floor vs table

The robot can be deliberately placed on the floor for comparison:

```bash
python vision_bridge_controlled.py --gui --controller --robot-mount floor --steps 3000
```

For normal operation, use `--robot-mount table` or rely on the default.

---

## 9. Rumble Intensity and Timing

The final bridge exposes:

| Option | Meaning | Default |
| --- | --- | --- |
| `--rumble-strength` | Motor strength from `0.0` to `1.0` | `0.5` |
| `--rumble-duration` | Initial-contact rumble duration in milliseconds | `100` |

Example with stronger contact feedback:

```bash
python vision_bridge_controlled.py --gui --controller --rumble-strength 0.8 --rumble-duration 200 --steps 3000
```

The grasp rumble uses twice the configured duration. For example, with `--rumble-duration 150`, a grasp requests approximately 300 milliseconds.

Use a lower intensity if the controller is uncomfortable:

```bash
python vision_bridge_controlled.py --gui --controller --rumble-strength 0.3 --rumble-duration 120 --steps 3000
```

Use values between `0.0` and `1.0`. A driver may ignore rumble or report it as unsupported even when controller axes and buttons work.


## 10. Troubleshooting

### 10.1 Conda / environment

#### 10.1.1 `conda` is not recognized on Windows

If `conda --version` produces `The term 'conda' is not recognized`, the shell has not been initialized. This is expected before `conda init`. Run:

```powershell
& "$env:LOCALAPPDATA\miniconda3\Scripts\conda.exe" init powershell
& "$env:LOCALAPPDATA\miniconda3\Scripts\conda.exe" config --set auto_activate_base false
```

Then close all PowerShell and Windows Terminal sessions and open a new PowerShell terminal. Verify with `conda --version` and `conda info --base`. If the Miniconda installation is in a different directory, adjust the path accordingly.


```powershell
where.exe conda
Get-Command conda
conda info --base
```

The base should refer to the intended Miniconda installation (typically `$env:LOCALAPPDATA\miniconda3`) rather than an old Anaconda directory. If it points to an old installation, uninstall that distribution or remove its `PATH` and shell-init entries.

#### 10.1.2 Wrong Conda installation is being used

```powershell
where.exe conda
Get-Command conda
conda info --base
```

The base should refer to the intended Miniconda installation (typically `$env:LOCALAPPDATA\miniconda3`) rather than an old Anaconda directory. If it points to an old installation, uninstall that distribution or remove its `PATH` and shell-init entries.

#### 10.1.3 Wrong Python is active

```powershell
where.exe python
python --version
conda info --envs
```

When `franka-haptics` is active, `python --version` should report `3.12.x` and `where.exe python` should point inside the `franka-haptics` environment. If it does not, ensure you ran `conda activate franka-haptics` and that no other manager is shadowing `python`.

On macOS/Linux use `which python` / `which python3` / `conda info --envs` instead of `where.exe`.

#### 10.1.4 `(base)` appears every time the terminal starts

The `base` environment is set to auto-activate. Disable it:

```bash
conda config --set auto_activate_base false
```

Restart the shell. You can still activate any environment explicitly with `conda activate franka-haptics`.

#### 10.1.5 Package installed into the wrong Python

```bash
python -m pip --version
```

This shows which Python `pip` belongs to. Prefer:

```bash
python -m pip install ...
```

over bare `pip install ...`, because `python -m pip` unambiguously uses pip from the currently active interpreter.

### 10.2 Simulation and controller

#### 10.2.1 `No controllers found`

Check the following:

1. The DualSense is connected by USB or Bluetooth.
2. The controller is charged and powered on.
3. Other applications are not holding the controller connection.
4. The `franka-haptics` Conda environment is active.
5. Pygame is installed in that environment.

Then run:

```bash
python controller_test_ps5.py --list
```

#### 10.2.2 Axes print correctly but the arm does not move

Run the controlled bridge with diagnostics:

```bash
python vision_bridge_controlled.py --gui --controller --debug-controller --print-every 30 --steps 1000
```

Move one stick at a time and check that the printed values change. Also verify that the command includes `--controller`.

If movement is too small, increase the speed:

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.018 --steps 1000
```

#### 10.2.3 The robot is on the floor

The controlled runner defaults to table mounting. Explicitly select the table:

```bash
python vision_bridge_controlled.py --gui --controller --robot-mount table --steps 1000
```

Do not confuse the controlled runner with older commands that may default to floor mounting.

#### 10.2.4 Rumble does not work

First test the controller separately:

```bash
python controller_test_ps5.py --rumble-test --rumble-on-button --duration 10
```

Press a controller button once. Some DualSense driver combinations begin responding to rumble only after an input event. If it still does not work, reconnect by USB and check Windows game-controller drivers.

The controller may support input while not supporting rumble through the current SDL/Pygame backend.

#### 10.2.5 `maxVelocities is an invalid keyword argument`

This project intentionally does not pass `maxVelocities` to `setJointMotorControlArray`, because some installed PyBullet versions reject that keyword. Use the project dependency environment and do not add that keyword back without checking the installed PyBullet API.

#### 10.2.6 OpenGL or GUI problems

Try TinyRenderer or safe mode:

```bash
python vision_bridge_controlled.py --gui --controller --safe-mode --steps 1000
```

You can also test without the GUI:

```bash
python vision_bridge_controlled.py --controller --tiny --steps 1000
```

#### 10.2.7 Camera windows do not appear

Use the PyBullet GUI first. OpenCV windows require a desktop session and are enabled with:

```bash
python vision_bridge_controlled.py --gui --use-overhead --cv2-view --steps 1000
```

#### 10.2.8 The arm collides with cubes immediately

Move the table-mounted base farther from the cube workspace:

```bash
python vision_bridge_controlled.py --gui --controller --robot-x 0.10 --steps 1000
```

The default controlled base position is already separated from the normal cube spawn area, but the arm's reachable workspace can still overlap the cubes depending on the joint pose and joystick commands.

---

## 11. Quick Command Reference

Activate the environment first -- all commands in this section assume `franka-haptics` is active:

```bash
conda activate franka-haptics
```

### Controller diagnostics

```bash
python controller_test_ps5.py --list
python controller_test_ps5.py --duration 10
python controller_test_ps5.py --rumble-test --duration 3
python controller_test_ps5.py --rumble-on-button --duration 10
```

### Normal controlled demo

```bash
python vision_bridge_controlled.py --gui --controller --use-overhead --steps 3000
```

### Fast controlled demo

```bash
python vision_bridge_controlled.py --gui --controller --control-speed 0.03 --physics-steps 4 --steps 3000
```

### Stronger rumble

```bash
python vision_bridge_controlled.py --gui --controller --rumble-strength 0.8 --rumble-duration 200 --steps 3000
```

### Controlled demo with both cameras

```bash
python vision_bridge_controlled.py --gui --controller --use-overhead --use-wrist --steps 3000
```

### Safe mode

```bash
python vision_bridge_controlled.py --gui --controller --safe-mode --steps 3000
```

### Vision-only demo

```bash
python vision_bridge.py --gui --use-overhead --use-wrist --detector seg --steps 1000
```

When finished:

```bash
conda deactivate
```
