# Quickstart: Franka Panda + Game Controller (Windows)

Three parts: **1. Setup** (once), **2. Run**, **3. Configure**.
The full reference is in `README.md`; this is the short version that has been tested.

---

## 1. Setup (once)

### 1.1 Install Miniconda
Download **Miniconda** (not Anaconda) from <https://www.anaconda.com/download/success> and run the installer with the defaults.

Find where it installed (usually `C:\Users\<you>\miniconda3` or `%LOCALAPPDATA%\miniconda3`):

```powershell
Get-Command conda | Select-Object Source
```

Below, `<CONDA>` means that folder, for example `C:\Users\Admin\miniconda3`.

### 1.2 Create the environment and install packages
Open PowerShell and run these three commands one at a time:

```powershell
& "<CONDA>\Scripts\conda.exe" create -y -n franka-haptics python=3.12 pip
& "<CONDA>\Scripts\conda.exe" install -y -n franka-haptics -c conda-forge "pybullet=3.25"
& "<CONDA>\envs\franka-haptics\python.exe" -m pip install "numpy==1.26.0" "opencv-python==4.10.0.82" "pillow==11.3.0" pygame
```

> The README says `pybullet==3.2.5`. That version does not exist; use `3.25`.
> You can skip `conda init` and `conda activate`: the commands below call the environment's Python directly.

### 1.3 Check it worked

```powershell
& "<CONDA>\envs\franka-haptics\python.exe" -c "import numpy, pybullet, cv2, PIL, pygame; print('OK')"
```

It should print `OK` (plus a few version lines).

### 1.4 Connect the controller
Use **USB cable or the wireless dongle**. Avoid plain Bluetooth.

| Controller | What to do |
| --- | --- |
| PS5 DualSense | Connect by USB or Bluetooth. |
| 8BitDo Ultimate 2 | Set the mode switch to **X (XInput)**, then connect by dongle or USB. Other modes may work for sticks but **not for rumble**. |
| Xbox-style pads | Normally work as-is. |

Test that it is seen (run from the `haptic-control` folder):

```powershell
cd "<path-to>\haptic-control"
& "<CONDA>\envs\franka-haptics\python.exe" controller_test_ps5.py --list
```

You should see a line such as `[0] ... axes=6 buttons=11 hats=1`. If it says `No controllers found`, reconnect and try again.

Test rumble (you should feel a 1 second buzz):

```powershell
& "<CONDA>\envs\franka-haptics\python.exe" controller_test_ps5.py --rumble-test --rumble-strength 1.0 --rumble-duration 1000 --duration 3
```

No buzz? See [Troubleshooting](#4-troubleshooting).

---

## 2. Run

From the `haptic-control` folder:

```powershell
& "<CONDA>\envs\franka-haptics\python.exe" vision_bridge_controlled.py --gui --controller --control-speed 0.03 --use-overhead --physics-steps 4 --steps 3000
```

A PyBullet window opens with the arm on a table and coloured cubes.

### Controls

| Input | Action |
| --- | --- |
| Left stick left/right | Move hand in X |
| Left stick up/down | Move hand in Y |
| Right stick up/down | Move hand in Z (up/down) |
| Button 0 (Cross / A) | Close gripper |
| Button 1 (Circle / B) | Open gripper |

### Feedback
- **Short buzz:** the hand or a finger touched a cube.
- **Longer buzz:** both fingers are touching the same cube (a grasp).

### Stop
Press **Ctrl+C** in the terminal. (Closing the window also works but prints a harmless error.)

`--steps 3000` ends the run after 3000 cycles. Omit it to run until you stop it.

---

## 3. Configure (tweak to preference)

Add these options to the run command. Change one at a time.

### Movement

| Option | Default | Effect |
| --- | --- | --- |
| `--control-speed` | `0.012` | Hand speed. Try `0.006` for fine control, `0.03` for fast. Sensible range `0.006` to `0.03`. |
| `--physics-steps` | `2` | Simulation steps per update. Raise to `3` or `4` with high speeds. If motion gets jumpy, go back to `2`. |

### Rumble

| Option | Default | Effect |
| --- | --- | --- |
| `--rumble-strength` | `0.5` | `0.0` (off) to `1.0` (max). |
| `--rumble-duration` | `100` | Touch buzz in milliseconds. Grasp buzz is twice this. |

### Scene

| Option | Default | Effect |
| --- | --- | --- |
| `--n_cubes` | `8` | Number of cubes. |
| `--robot-x`, `--robot-y` | `0.15`, `0.0` | Robot base position on the table. Move it back (smaller X) if the arm hits cubes at the start. |
| `--robot-mount` | `table` | `table` or `floor`. |

### Cameras and display

| Option | Effect |
| --- | --- |
| `--use-overhead` / `--use-wrist` | Turn on the overhead / wrist camera. |
| `--detector seg` | Use the segmentation detector (default is `stub`). |
| `--save output` | Save camera frames to the `output` folder. |
| `--cv2-view` | Show camera streams in extra OpenCV windows. |
| `--safe-mode` | Low-resolution CPU rendering for slow PCs or graphics problems. |

### Debugging

| Option | Effect |
| --- | --- |
| `--debug-controller --print-every 30` | Print stick values every 30 steps, to check which axis does what. |
| `--controller-index N` | Use controller number `N` if several are connected. |

### Example presets

Gentle and precise:
```
... --control-speed 0.006 --rumble-strength 0.3 --steps 3000
```

Fast, strong feedback:
```
... --control-speed 0.03 --physics-steps 4 --rumble-strength 1.0 --rumble-duration 200 --steps 3000
```
(`...` = `vision_bridge_controlled.py --gui --controller`, as in the run command.)

---

## 4. Troubleshooting

| Problem | Fix |
| --- | --- |
| `No controllers found` | Reconnect it, close other apps using it, re-run `--list`. |
| Arm does not move | Make sure `--controller` is in the command. Run with `--debug-controller --print-every 30` and check the axis values change when you move the sticks. |
| Arm moves too little/much | Change `--control-speed`. |
| Sticks do the wrong thing | The axis layout is the standard one. Check the debug output. Axes 0/1 are the left stick and 2/3 the right stick. |
| No rumble (the command says it worked) | The controller is probably not in **XInput** mode, or is on plain Bluetooth. Switch to XInput + dongle/USB. Check vibration is not turned off in the controller's own software. |
| Window is blank or slow | Add `--safe-mode`. |
| `Not connected to physics server` on exit | Harmless. It appears when the window is closed. Use Ctrl+C next time. |
| `conda` not recognised | You do not need it: use the full path to `python.exe` as shown above. |

The terminal prints a `[contact] touch -> True` line whenever a rumble is sent. `True` means the command was accepted. If you see it but feel nothing, the issue is the controller mode or connection, not the code.
