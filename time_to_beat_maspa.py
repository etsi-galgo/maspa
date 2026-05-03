"""
time_to_beat_maspa.py

Run each RRT variant ONCE per scenario with no fixed budget.
Stop when it first outperforms MASPA or after MAX_TIME seconds.
Report the exact elapsed second when MASPA was beaten, or DNF.

Sequential (S3): T1 runs for T1_BUDGET (reliably solved), then T2 gets
the remaining time.  Total elapsed = T1_BUDGET + T2_beat_time.
"""

import contextlib, os, pickle as pkl
from run_benchmark import (RRT_star, Informed_RRT_star, Smart_RRT_star,
                           dijkstra, scenario_board_shape)
from path_lengths import ground_length, tether_length

MAX_TIME  = 600    # 1 minute
T1_BUDGET = 20    # seconds for T1 in sequential (always solved well within this)
k_length  = 26
radius    = 10**6

RRT_DEFS = [("RRT*", "rrt"), ("Informed RRT*", "informed"), ("Smart RRT*", "smart")]


def _run(algo_key, S, T, gobs, aobs, bshape, budget):
    with open(os.devnull, "w") as dn, contextlib.redirect_stdout(dn):
        if algo_key == "rrt":
            G = RRT_star(S[:2], T, gobs, aobs, radius, k_length,
                         n_iter=10**6, time_for_ending=budget, board_shape=bshape)
        elif algo_key == "informed":
            G = Informed_RRT_star(S[:2], T, gobs, aobs, radius, k_length,
                                  time_for_ending=budget, board_shape=bshape)
        else:
            G = Smart_RRT_star(S[:2], T, gobs, aobs, radius, k_length,
                               time_for_ending=budget, board_shape=bshape)
    return G


def _first_beat(log, threshold):
    """First elapsed time where cost < threshold, or None."""
    for (t, cost, _) in log:
        if cost < threshold:
            return t
    return None


def _maspa_ref(pd):
    if pd["sequential"]:
        segs  = pd["methods"]["MASPA"]["segments"]
        total = 0.0
        for i, seg in enumerate(segs):
            g = ground_length(seg["ground_path"])
            a = tether_length(seg["tether"])
            total += g + (1 if i == len(segs) - 1 else 2) * a
        return total
    else:
        md = pd["methods"]["MASPA"]
        return ground_length(md["ground_path"]) + tether_length(md["tether"])


def beat_single(algo_key, S, T, gobs, aobs, bshape, maspa_ref):
    G = _run(algo_key, S, T, gobs, aobs, bshape, MAX_TIME)
    return _first_beat(G._improve_log, maspa_ref)


def beat_sequential(algo_key, S_init, Ts, gobs, aobs, bshape, maspa_ref):
    G1 = _run(algo_key, S_init, Ts[0], gobs, aobs, bshape, T1_BUDGET)
    if not G1.success or not G1._improve_log:
        return None

    _, best_lc, best_ll = G1._improve_log[-1]   # last = best (log is monotone)
    cost_offset = best_lc + best_ll              # d(S→X1) + 2·d(X1→T1)

    gpath, _ = dijkstra(G1, Ts[0])
    S_cur = tuple([*gpath[-1][:2], 0])

    G2 = _run(algo_key, S_cur, Ts[1], gobs, aobs, bshape, MAX_TIME - T1_BUDGET)

    for (lt, lc, _) in G2._improve_log:
        if cost_offset + lc < maspa_ref:
            return T1_BUDGET + lt

    return None


# ── main ──────────────────────────────────────────────────────────────────────

SCENARIOS = [("S2", False), ("S3", True), ("S5", False)]

print(f"\n{'='*56}")
print(f"  Time to first beat MASPA  (1 run, {MAX_TIME}s limit)")
print(f"  Sequential T1 budget: {T1_BUDGET}s  |  T2 budget: {MAX_TIME-T1_BUDGET}s")
print(f"{'='*56}")

for sname, sequential in SCENARIOS:
    cache = f"scenarios/{sname}_plot_data.pkl"
    if not os.path.exists(cache):
        print(f"\n  [{sname}] run plot_comparison.py first to generate cache")
        continue
    with open(cache, "rb") as f:
        pd = pkl.load(f)
    if "MASPA" not in pd["methods"]:
        print(f"\n  [{sname}] MASPA not in cache")
        continue

    maspa_ref = _maspa_ref(pd)

    with open(f"scenarios/{sname}.pkl", "rb") as f:
        sc = pkl.load(f)
    S    = sc["S"]
    T    = sc["T"]
    gobs = sc["ground_obstacles"]
    aobs = sc["aerial_obstacles"]
    bshape = scenario_board_shape(sc)

    print(f"\n  {sname}  —  MASPA cost: {maspa_ref:.2f}")
    print(f"  {'-'*44}")

    for aname, akey in RRT_DEFS:
        print(f"    {aname:<16}  running...", end="\r", flush=True)
        if sequential:
            t = beat_sequential(akey, S, T, gobs, aobs, bshape, maspa_ref)
        else:
            t = beat_single(akey, S, T, gobs, aobs, bshape, maspa_ref)

        if t is not None:
            result = f"beat MASPA at  {t:6.1f} s"
        else:
            result = f"DNF  (not beaten in {MAX_TIME}s)"
        print(f"    {aname:<16}  {result}")

print(f"\n{'='*56}")
