"""
3-D comparison plot: MASPA vs RRT* variants for S2, S3, S5.

Results are cached in scenarios/<name>_plot_data.pkl so the first run
computes the paths; subsequent runs just load and plot.
Delete the cache file to force a fresh computation.

Ground path  : solid line  (─)
Aerial/tether: dashed line + x markers (──✕)
"""

import contextlib, os, pickle as pkl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

from maspa_planning import path_planning_smpp
from run_benchmark import (RRT_star, Informed_RRT_star, Smart_RRT_star,
                            dijkstra, scenario_board_shape)

COLORS = {
    "MASPA":         "#2196F3",  # blue
    "RRT*":          "#FF9800",  # orange
    "Informed RRT*": "#4CAF50",  # green
    "Smart RRT*":    "#E91E63",  # pink
}

# ── helpers ───────────────────────────────────────────────────────────────────

def _run_rrt(algo_key, S, T, gobs, aobs, bshape, budget):
    with open(os.devnull, "w") as dn, contextlib.redirect_stdout(dn):
        if algo_key == "rrt":
            G = RRT_star(S[:2], T, gobs, aobs, 10**6, 26,
                         n_iter=10**6, time_for_ending=budget, board_shape=bshape)
        elif algo_key == "informed":
            G = Informed_RRT_star(S[:2], T, gobs, aobs, 10**6, 26,
                                  time_for_ending=budget, board_shape=bshape)
        else:
            G = Smart_RRT_star(S[:2], T, gobs, aobs, 10**6, 26,
                               time_for_ending=budget, board_shape=bshape)
    return G


def _extract_rrt(G, T):
    """Return (ground_path, tether) from a solved Graph, or (None, None)."""
    if not G.success or G.opt_cat is None:
        return None, None
    gpath, _ = dijkstra(G, T)
    return gpath, G.opt_cat


# ── data collection ───────────────────────────────────────────────────────────

RRT_DEFS = [("RRT*", "rrt"), ("Informed RRT*", "informed"), ("Smart RRT*", "smart")]


def collect_single(sname, budget=40):
    """Run or load all 4 methods for a single-target scenario."""
    cache = f"scenarios/{sname}_plot_data.pkl"
    if os.path.exists(cache):
        print(f"  [cache] {cache}")
        with open(cache, "rb") as f:
            return pkl.load(f)

    with open(f"scenarios/{sname}.pkl", "rb") as f:
        sc = pkl.load(f)
    S, T = sc["S"], sc["T"]
    gobs, aobs = sc["ground_obstacles"], sc["aerial_obstacles"]
    bshape = scenario_board_shape(sc)
    methods = {}

    print(f"  [MASPA] ...", flush=True)
    with open(os.devnull, "w") as dn, contextlib.redirect_stdout(dn):
        res = path_planning_smpp(S, T, gobs, aobs, p=16, q=30, k_length=26)
    if res[0] is not None:
        gpath, (anchor3d, ctop_d, _), _, _ = res
        methods["MASPA"] = {"ground_path": gpath,
                            "tether":      ctop_d[anchor3d]["tether"]}

    for aname, akey in RRT_DEFS:
        print(f"  [{aname}] {budget}s ...", flush=True)
        G = _run_rrt(akey, S, T, gobs, aobs, bshape, budget)
        gpath, tether = _extract_rrt(G, T)
        if gpath is not None:
            methods[aname] = {"ground_path": gpath, "tether": tether}

    data = {"S": S, "T": T, "gobs": gobs, "aobs": aobs,
            "sequential": False, "methods": methods}
    with open(cache, "wb") as f:
        pkl.dump(data, f)
    print(f"  Saved: {cache}")
    return data


def collect_sequential(sname, budget_per_target=20):
    """Run or load all 4 methods for a sequential-target scenario."""
    cache = f"scenarios/{sname}_plot_data.pkl"
    if os.path.exists(cache):
        print(f"  [cache] {cache}")
        with open(cache, "rb") as f:
            return pkl.load(f)

    with open(f"scenarios/{sname}.pkl", "rb") as f:
        sc = pkl.load(f)
    S_init = sc["S"]
    Ts     = sc["T"]
    gobs, aobs = sc["ground_obstacles"], sc["aerial_obstacles"]
    bshape = scenario_board_shape(sc)
    methods = {}

    print(f"  [MASPA] {len(Ts)} targets ...", flush=True)
    S_cur, vis, segs = S_init, None, []
    for T in Ts:
        with open(os.devnull, "w") as dn, contextlib.redirect_stdout(dn):
            res = path_planning_smpp(S_cur, T, gobs, aobs,
                                     p=16, q=30, k_length=26, visibility=vis)
        if res[0] is None:
            break
        gpath, (anchor3d, ctop_d, _), _, vis = res
        segs.append({"ground_path": gpath, "tether": ctop_d[anchor3d]["tether"]})
        S_cur = tuple([*anchor3d[:2], 0])
    if segs:
        methods["MASPA"] = {"segments": segs}

    for aname, akey in RRT_DEFS:
        print(f"  [{aname}] {budget_per_target}s × {len(Ts)} ...", flush=True)
        S_cur, segs = S_init, []
        for T in Ts:
            G = _run_rrt(akey, S_cur, T, gobs, aobs, bshape, budget_per_target)
            gpath, tether = _extract_rrt(G, T)
            if gpath is None:
                break
            segs.append({"ground_path": gpath, "tether": tether})
            S_cur = tuple([*gpath[-1][:2], 0])
        if segs:
            methods[aname] = {"segments": segs}

    data = {"S": S_init, "Ts": Ts, "gobs": gobs, "aobs": aobs,
            "sequential": True, "methods": methods}
    with open(cache, "wb") as f:
        pkl.dump(data, f)
    print(f"  Saved: {cache}")
    return data


# ── 3-D plotting ──────────────────────────────────────────────────────────────

def _add_obstacles(ax, obs_list):
    for pts in obs_list:
        pts = np.array(pts)
        if len(pts) < 4:
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c="gray", s=4)
            continue
        try:
            hull = ConvexHull(pts)
            for s in hull.simplices:
                s = np.append(s, s[0])
                verts = [list(zip(pts[s, 0], pts[s, 1], pts[s, 2]))]
                ax.add_collection3d(Poly3DCollection(
                    verts, alpha=0.25,
                    facecolor="slategray", edgecolor="dimgray", linewidth=0.6))

        except Exception:
            ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c="gray", s=4)


def _draw_segment(ax, seg, color, label=None):
    gp     = seg["ground_path"]
    tether = seg["tether"]

    xs = [p[0] for p in gp]
    ys = [p[1] for p in gp]
    ax.plot(xs, ys, [0]*len(gp), "-", color=color, linewidth=2.5, label=label)

    if tether is not None and len(tether):
        t = np.asarray(tether)
        ax.plot(t[:, 0], t[:, 1], t[:, 2], "--", color=color, linewidth=1.2, alpha=0.8)
        ax.plot(t[:, 0], t[:, 1], t[:, 2], "x",  color=color, markersize=5)


def make_figure(data):
    fig = plt.figure(figsize=(13, 9))
    ax  = fig.add_subplot(111, projection="3d")

    _add_obstacles(ax, data["gobs"] + data["aobs"])

    if data["sequential"]:
        S, Ts = data["S"], data["Ts"]
        ax.plot([S[0]], [S[1]], [S[2]], "ko", markersize=9, label="S", zorder=5)
        for i, T in enumerate(Ts):
            ax.plot([T[0]], [T[1]], [T[2]], "r*", markersize=12,
                    label=f"T{i+1}", zorder=5)
        for mname, mdata in data["methods"].items():
            color = COLORS.get(mname, "k")
            for j, seg in enumerate(mdata["segments"]):
                _draw_segment(ax, seg, color, label=(mname if j == 0 else None))
    else:
        S, T = data["S"], data["T"]
        ax.plot([S[0]], [S[1]], [S[2]], "ko", markersize=9, label="S", zorder=5)
        ax.plot([T[0]], [T[1]], [T[2]], "r*", markersize=12, label="T", zorder=5)
        for mname, mdata in data["methods"].items():
            color = COLORS.get(mname, "k")
            _draw_segment(ax, mdata, color, label=mname)

    ax.set_xlabel(""); ax.set_ylabel(""); ax.set_zlabel("")
    ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    
    # leg = ax.legend(
    #     loc="center",
    #     bbox_to_anchor=(0.2, 0.7),
    #     fontsize=18,
    #     frameon=True
    # )

    # frame = leg.get_frame()
    # frame.set_alpha(1)
    # frame.set_facecolor("white")
    # frame.set_edgecolor("black")
    # frame.set_zorder(1000)

    # leg.set_zorder(1000)

    ax.set_title("")
    fig.tight_layout()
    return fig, ax


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    configs = [
        ("S2", False, 10),
        ("S3", True,  10),   # 20 s per target
        ("S5", False, 10),
    ]

    for sname, sequential, budget in configs:
        print(f"\n=== {sname} ===")
        if sequential:
            result = collect_sequential(sname, budget_per_target=budget)
        else:
            result = collect_single(sname, budget=budget)

        print(f"  Methods: {list(result['methods'].keys())}")
        fig, ax = make_figure(result)
        savepath = f"scenarios/{sname}_comparison.png"
        fig.savefig(savepath, dpi=150, bbox_inches="tight")
        print(f"  PNG: {savepath}")

    print("\nRotate to find best perspective, then close all windows.")
    plt.show()
