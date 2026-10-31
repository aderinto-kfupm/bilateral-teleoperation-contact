#!/usr/bin/env python3
"""Phase 3 pilot experiments: is the task neither trivial nor impossible?

The scripted operator pushes its reference a set depth past the surface while tracing the path.
Sweeping that depth shows how precisely an operator must control the normal direction:

  * baseline (direct position teleop, visual only) passes only inside a narrow depth window, so the
    task is not trivial;
  * it does pass inside that window, and scenario C passes at every depth, so it is not impossible.

  python -m teleop.pilot                 # default sweep, about 45 s headless
  python -m teleop.pilot --seeds 3       # repeat each condition with 3 tremor seeds

Writes results/pilot_<stamp>/: pilot.csv (one row per run), pilot_summary.csv, pilot.png.
"""
import argparse
import csv
import os
import time
from types import SimpleNamespace

import numpy as np

from . import config
from .run import F_FAIL, metrics, run_scenario

DEPTHS_MM = [0, 1, 2, 3, 4, 5, 7.5, 10, 20, 30]


def one(name, operator, depth_mm, seed):
    args = SimpleNamespace(gui=False, operator=operator, seed=seed, push_depth=depth_mm / 1000.0,
                           controller_index=0, clutch_button=None)
    rows, P = run_scenario(name, args)
    m = metrics(name, rows, P)
    return dict(scenario=name, operator=operator, depth_mm=depth_mm, seed=seed, success=int(m["success"]),
                fail_reason=m["fail_reason"], fn_peak=m["fn_peak"], fn_mean=m["fn_mean"],
                contact_loss_max_s=m["contact_loss_max_s"], coverage_pct=m["coverage_pct"])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=1,
                    help="seeds per condition (the seed only changes the tangential tremor, so 1 is enough)")
    ap.add_argument("--scenarios", nargs="+", default=["baseline", "C"])
    ap.add_argument("--outdir", default=None)
    a = ap.parse_args()
    outdir = a.outdir or str(config.RESULTS / ("pilot_" + time.strftime("%Y%m%d_%H%M%S")))
    os.makedirs(outdir, exist_ok=True)

    runs = []
    for name in a.scenarios:
        for d in DEPTHS_MM:
            for s in range(a.seeds):
                r = one(name, "scripted", d, s)
                runs.append(r)
                print(f"{name:9s} depth={d:5.1f} mm seed={s}  pass={r['success']}  peak={r['fn_peak']} N  {r['fail_reason']}")
    for name in ("baseline", "A", "B", "C"):      # operator that reacts to the cue, default 30 mm intent
        for s in range(a.seeds):
            r = one(name, "reactive", 30, s)
            runs.append(r)
            print(f"{name:9s} reactive        seed={s}  pass={r['success']}  peak={r['fn_peak']} N  {r['fail_reason']}")

    with open(os.path.join(outdir, "pilot.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0]))
        w.writeheader()
        w.writerows(runs)

    groups = {}
    for r in runs:
        groups.setdefault((r["scenario"], r["operator"], r["depth_mm"]), []).append(r)
    summ = [dict(scenario=k[0], operator=k[1], depth_mm=k[2], runs=len(v),
                 pass_rate=round(np.mean([r["success"] for r in v]), 2),
                 fn_peak_median=round(float(np.median([r["fn_peak"] for r in v])), 2))
            for k, v in groups.items()]
    with open(os.path.join(outdir, "pilot_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for name in a.scenarios:
        rs = [s for s in summ if s["scenario"] == name and s["operator"] == "scripted"]
        ax.plot([s["depth_mm"] for s in rs], [s["fn_peak_median"] for s in rs], "o-", label=name)
    ax.axhline(F_FAIL, color="r", ls=":", lw=1, label=f"failure ({F_FAIL:.0f} N)")
    ax.set_xscale("symlog", linthresh=1)
    ax.set_xlim(-0.3, 35)
    ax.set_xlabel("operator push depth past the surface [mm]")
    ax.set_ylabel("peak normal force [N]")
    ax.set_yscale("log")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "pilot.png"), dpi=130)
    print("saved to", outdir)


if __name__ == "__main__":
    main()
