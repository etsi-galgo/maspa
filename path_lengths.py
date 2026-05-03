"""
Compute total path lengths from *_plot_data.pkl cache files.

Single target  : L = d(S→X)  +   d(X→T)
Sequential     : L = d(S→X1) + 2·d(X1→T1) + d(X1→X2) + 2·d(X2→T2)
"""

import os, pickle as pkl
import numpy as np


def ground_length(path):
    pts = [np.array(p[:2], dtype=float) for p in path]
    return sum(np.linalg.norm(pts[i+1] - pts[i]) for i in range(len(pts)-1))


def tether_length(tether):
    t = np.asarray(tether, dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(t, axis=0), axis=1)))


def total_length(method_data, sequential):
    if sequential:
        total = 0.0
        segs = method_data["segments"]
        breakdown = []
        for i, seg in enumerate(segs):
            is_last = (i == len(segs) - 1)
            g = ground_length(seg["ground_path"])
            a = tether_length(seg["tether"])
            multiplier = 1 if is_last else 2   # return trip only for non-last targets
            total += g + multiplier * a
            breakdown.append((g, a, multiplier))
        return total, breakdown
    else:
        g = ground_length(method_data["ground_path"])
        a = tether_length(method_data["tether"])
        return g + a, [(g, a, 1)]


SCENARIOS = ["S2", "S3", "S5"]

if __name__ == "__main__":
    for sname in SCENARIOS:
        cache = f"scenarios/{sname}_plot_data.pkl"
        if not os.path.exists(cache):
            print(f"[{sname}] not found — run  python plot_comparison.py  first\n")
            continue

        with open(cache, "rb") as f:
            data = pkl.load(f)

        seq     = data["sequential"]
        n_segs  = len(data["Ts"]) if seq else 1
        formula = ("d(S→X1) + 2·d(X1→T1) + d(X1→X2) + 2·d(X2→T2)"
                   if seq else "d(S→X) + d(X→T)")

        print(f"\n{'='*72}")
        print(f"  Scenario {sname}  {'[sequential]' if seq else '[single target]'}")
        print(f"  {formula}")
        print(f"{'='*72}")

        header = f"  {'Algorithm':<16}  {'TOTAL':>8}"
        if seq:
            for i in range(n_segs):
                mult = "1×" if i == n_segs - 1 else "2×"
                header += f"  {'gnd'+str(i+1):>7}  {mult+'teth'+str(i+1):>9}"
        else:
            header += f"  {'ground':>8}  {'tether':>8}"
        print(header)
        print(f"  {'-'*68}")

        for mname, mdata in data["methods"].items():
            length, breakdown = total_length(mdata, seq)
            row = f"  {mname:<16}  {length:>8.2f}"
            for g, a, mult in breakdown:
                row += f"  {g:>7.2f}  {mult}×{a:>7.2f}" if seq else f"  {g:>8.2f}  {a:>8.2f}"
            print(row)

        print()
