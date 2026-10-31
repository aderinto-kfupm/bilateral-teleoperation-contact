.PHONY: paper watch check env test lint sim reproduce pilot figures clean

# Use the project venv if `make env` created one (Windows: Scripts, else bin).
VENV_PY := $(firstword $(wildcard .venv/Scripts/python.exe .venv/bin/python))
PY      := $(if $(VENV_PY),$(VENV_PY),python)

paper:
	cd writeup && latexmk -pdf -interaction=nonstopmode -halt-on-error main.tex

watch:
	cd writeup && latexmk -pdf -pvc main.tex

# Fails on undefined citations/references. Run after `make paper`.
check:
	@! grep -E "undefined|Citation .* undefined|Reference .* undefined" writeup/main.log

env:
	python -m venv .venv
	$(or $(wildcard .venv/Scripts/python.exe),.venv/bin/python) -m pip install -e "src/python[dev]"

test:
	$(PY) -m pytest

lint:
	$(PY) -m ruff check src/python tests

sim:
	./scripts/sim.sh

# Headless run of every scenario; writes results/<stamp>/.
reproduce:
	$(PY) -m teleop.run --scenario all --operator scripted

# Phase 3 pilot sweep; writes results/pilot_<stamp>/.
pilot:
	PYTHONPATH=src/python $(PY) -m teleop.pilot

# Offscreen render of the scenario for the reports.
figures:
	PYTHONPATH=src/python $(PY) -m teleop.render

clean:
	cd writeup && latexmk -C
