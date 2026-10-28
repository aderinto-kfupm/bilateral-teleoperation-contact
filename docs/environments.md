# Environments

## Python
- Version: 3.10+. Dependencies are declared in `src/python/pyproject.toml` (runtime: numpy, matplotlib, pybullet; dev: pytest, ruff).
- Setup: `make env`, then activate `.venv`.
- Entry point: `python -m teleop.run` (writes `results/<stamp>/`). `make sim` / `scripts/sim.cmd` wrap it.
- Hardware env: the `franka-haptics` conda env from `src/haptic-control/QUICKSTART.md` (keep `numpy==1.26.0`, add `matplotlib==3.9.2`); the launchers use it, override with `SIM_PY`.
- Pin exact versions for the camera-ready: `pip freeze > requirements.lock.txt` and commit it.

## LaTeX
- Class `IEEEtran`, BibTeX. Build: `make paper`. CI builds on every push.
- Switch `[conference]` to `[journal]` in `writeup/main.tex` for transactions.
