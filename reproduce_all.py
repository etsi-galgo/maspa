"""
reproduce_all.py — one-shot script to reproduce all paper results.

Steps executed in order:
  1. Generate scenarios (skipped if .pkl files already exist)
  2. Run MASPA (SMPP and BF) on S2
  3. Run MASPA sequential on S3
  4. Run RRT benchmark (RRT*, Informed RRT*, Smart RRT*) on S2, S3, S5
  5. Generate 3-D comparison plots and save PNGs
  6. Print path-length table
  7. Run random-experiment metrics
"""

import os, sys, contextlib, pickle as pkl
import numpy as np

# ── 1. Generate scenarios if missing ─────────────────────────────────────────

SCENARIO_FILES = ["scenarios/S2.pkl", "scenarios/S3.pkl", "scenarios/S5.pkl"]

missing = [p for p in SCENARIO_FILES if not os.path.exists(p)]
if missing:
    print("==> Generating scenarios:", missing)
    import generate_scenarios  # noqa: F401 — runs generation at import time
else:
    print("==> Scenarios already present, skipping generation.")

# ── 2. MASPA on S2 (single target) ──────────────────────────────────────────

print("\n==> Running MASPA and MASPA-BF on S2 ...")
from maspa_planning import path_planning_smpp, path_planning_bf

with open("scenarios/S2.pkl", "rb") as f:
    s2 = pkl.load(f)

S, T = s2["S"], s2["T"]
gobs, aobs = s2["ground_obstacles"], s2["aerial_obstacles"]
p, q, k = 16, 30, 26

print("  [MASPA-SMPP]", flush=True)
with contextlib.redirect_stdout(sys.stdout):
    path_planning_smpp(S, T, gobs, aobs, p, q, k, plot=False)

print("  [MASPA-BF]", flush=True)
with contextlib.redirect_stdout(sys.stdout):
    path_planning_bf(S, T, gobs, aobs, p, q, k, plot=False)

# ── 3. MASPA sequential on S3 ────────────────────────────────────────────────

print("\n==> Running MASPA sequential on S3 ...")
from maspa_planning import maspa_sequential
maspa_sequential(plot=False)

# ── 4. RRT benchmark ─────────────────────────────────────────────────────────

print("\n==> Running RRT benchmark (RRT*, Informed RRT*, Smart RRT*) ...")
# N_RUNS=1 gives a quick single-run result; set to 30 to replicate the paper's statistics.
N_RUNS = 1
print(f"    Scenarios: S2, S3, S5  |  {N_RUNS} run(s) × 20s each")
print("    This will take a few minutes.\n")
from run_benchmark import run_benchmark
run_benchmark(n_runs=N_RUNS)

# ── 5. Generate comparison plots ─────────────────────────────────────────────

print("\n==> Generating 3-D comparison plots ...")
import plot_comparison

configs = [
    ("S2", False, 20),
    ("S3", True,  20),
    ("S5", False, 20),
]

for sname, sequential, budget in configs:
    print(f"\n=== {sname} ===")
    cache = f"scenarios/{sname}_plot_data.pkl"
    if os.path.exists(cache):
        print(f"  [cache] {cache}")
        with open(cache, "rb") as f:
            result = pkl.load(f)
    elif sequential:
        result = plot_comparison.collect_sequential(sname, budget_per_target=budget)
    else:
        result = plot_comparison.collect_single(sname, budget=budget)

    print(f"  Methods: {list(result['methods'].keys())}")
    fig, ax = plot_comparison.make_figure(result)
    savepath = f"scenarios/{sname}_comparison.png"
    fig.savefig(savepath, dpi=150, bbox_inches="tight")
    print(f"  PNG saved: {savepath}")

# ── 6. Path-length table ──────────────────────────────────────────────────────

print("\n==> Path-length summary:")
import path_lengths  # noqa: F401 — prints table when run as module, but it uses __main__

from path_lengths import ground_length, tether_length, total_length, SCENARIOS

for sname in SCENARIOS:
    cache = f"scenarios/{sname}_plot_data.pkl"
    if not os.path.exists(cache):
        print(f"[{sname}] cache not found — skipping")
        continue
    with open(cache, "rb") as f:
        data = pkl.load(f)
    seq = data["sequential"]
    formula = ("d(S→X1)+2·d(X1→T1)+d(X1→X2)+2·d(X2→T2)"
               if seq else "d(S→X)+d(X→T)")
    print(f"\n{'='*70}")
    print(f"  Scenario {sname}  {'[sequential]' if seq else '[single target]'}")
    print(f"  Formula: {formula}")
    print(f"{'='*70}")
    print(f"  {'Algorithm':<16}  {'TOTAL':>8}")
    for mname, mdata in data["methods"].items():
        length, _ = total_length(mdata, seq)
        print(f"  {mname:<16}  {length:>8.2f}")

# ── 7. Random-experiment metrics ──────────────────────────────────────────────

RANDOM_RESULTS = "scenarios/random_results_final.pkl"
if os.path.exists(RANDOM_RESULTS):
    print("\n==> Random-experiment metrics (MASPA, 1000 instances):")
    from metrics import compute_metrics_random_exp2  # noqa: E402
    compute_metrics_random_exp2()
else:
    print(f"\n==> {RANDOM_RESULTS} not found; skipping random metrics.")
    print("    To run random experiments, call  maspa_planning.run_random_experiments()  manually.")

print("\n==> Done. All results reproduced.")
