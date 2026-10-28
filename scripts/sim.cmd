@echo off
rem One-command launcher for the teleoperation simulation (works from cmd and PowerShell).
rem   scripts\sim                      -> self-check, then GUI run of scenario C (the paper's method)
rem   scripts\sim <any teleop.run args>    -> passed straight through, e.g.  sim --scenario all --operator scripted
rem   scripts\sim --scenario X --gui --operator dualsense
rem Set SIM_PY to use a different Python.
setlocal
cd /d "%~dp0.."
set "PYTHONPATH=%CD%\src\python"

if not defined SIM_PY set "SIM_PY=%USERPROFILE%\miniconda3\envs\franka-haptics\python.exe"
if not exist "%SIM_PY%" set "SIM_PY=%LOCALAPPDATA%\miniconda3\envs\franka-haptics\python.exe"
if not exist "%SIM_PY%" (
  echo [sim] franka-haptics env not found. Set it up per src\haptic-control\QUICKSTART.md or set SIM_PY=path\to\python.exe
  exit /b 1
)

"%SIM_PY%" -c "import numpy, pybullet, matplotlib" 2>nul
if errorlevel 1 (
  echo [sim] missing packages in %SIM_PY%
  echo [sim] fix: "%SIM_PY%" -m pip install numpy==1.26.0 matplotlib==3.9.2 pybullet
  exit /b 1
)

echo [sim] self-check...
"%SIM_PY%" tests	est_controller.py
if errorlevel 1 (
  echo [sim] self-check FAILED, not starting the simulation
  exit /b 1
)

if "%~1"=="" (
  "%SIM_PY%" -m teleop.run --scenario C --operator scripted --gui
) else (
  "%SIM_PY%" -m teleop.run %*
)
exit /b %errorlevel%
