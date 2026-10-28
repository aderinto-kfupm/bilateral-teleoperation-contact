#!/usr/bin/env bash
# One-command launcher for the teleoperation simulation (Git Bash, Linux, macOS).
#   ./scripts/sim.sh                      -> self-check, then GUI run of scenario C (the paper's method)
#   ./scripts/sim.sh <any teleop.run args>    -> passed straight through, e.g.  ./scripts/sim.sh --scenario all --operator scripted
# Set SIM_PY to use a specific Python.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export PYTHONPATH="$PWD/src/python"

find_py() {
  [ -n "$SIM_PY" ] && { echo "$SIM_PY"; return; }
  local c
  for c in "$HOME/miniconda3/envs/franka-haptics/python.exe" "$HOME/miniconda3/envs/franka-haptics/bin/python" \
           "$LOCALAPPDATA/miniconda3/envs/franka-haptics/python.exe" "$HOME/anaconda3/envs/franka-haptics/bin/python"; do
    [ -x "$c" ] && { echo "$c"; return; }
  done
  command -v python3 || command -v python || true
}

PY="$(find_py)"
if [ -z "$PY" ]; then
  echo "[sim] no Python found. Set up the env per src/haptic-control/QUICKSTART.md or export SIM_PY=/path/to/python" >&2
  exit 1
fi

if ! "$PY" -c "import numpy, pybullet, matplotlib" 2>/dev/null; then
  echo "[sim] missing packages in $PY" >&2
  echo "[sim] fix: \"$PY\" -m pip install numpy==1.26.0 matplotlib==3.9.2 pybullet" >&2
  exit 1
fi

echo "[sim] self-check..."
"$PY" tests/test_controller.py || { echo "[sim] self-check FAILED, not starting the simulation" >&2; exit 1; }

if [ $# -eq 0 ]; then
  exec "$PY" -m teleop.run --scenario C --operator scripted --gui
else
  exec "$PY" -m teleop.run "$@"
fi
