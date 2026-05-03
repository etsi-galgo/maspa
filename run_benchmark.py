'''
MIT License
Copyright (c) 2019 Fanjin Zeng
This work is licensed under the terms of the MIT license, see <https://opensource.org/licenses/MIT>.
'''

import matplotlib.pyplot as plt
import numpy as np
import pickle as pkl
import time

from collections import deque
from matplotlib import collections  as mc
from random import random
from scipy import linalg

from cat2 import *
from constants import *
from tools import euclidian_distance


BOARD_SHAPE = (100, 70, 40)


def scenario_board_shape(data, margin=20):
    """Compute a sampling box that covers start, all targets, and all obstacles."""
    points = [data["S"][:2]]
    T = data["T"]
    targets = T if isinstance(T, list) else [T]
    for t in targets:
        points.append(t[:2])
    for obs in data.get("ground_obstacles", []):
        for v in obs:
            points.append(v[:2])
    pts = np.array(points)
    dX = float(np.abs(pts[:, 0]).max()) + margin
    dY = float(np.abs(pts[:, 1]).max()) + margin
    return (dX, dY, BOARD_SHAPE[2])


class Graph:
    ''' Define graph '''
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {tuple(startpos):0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

        self.opt_cat = []
        self.opt_length = None
        self.opt_total_length = None

        self._improve_log = []   # (elapsed_seconds, total_cost) on each improvement
        self._init_time   = None

    def get_id(self, node):
        return self.vex2idx[tuple(node)]

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[tuple(pos)]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[tuple(pos)] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))

    def randomPosition(self, board_shape, obstacles, radius):

        deltaX, deltaY = board_shape
        while True:
            random_point = np.random.rand(2)-0.5
            random_point[0] *= 2*deltaX
            random_point[1] *= 2*deltaY

            distances = []
            new_edges = []
            for node in self.vertices:
                d = euclidian_distance(node, random_point)
                if d <= radius:
                    idx = self.get_id(node)
                    distances.append(d+self.distances[idx])
                    new_edges.append((node, random_point))

            items = sorted(zip(distances, new_edges), key=lambda x: x[0])
            for d, new_edge in items:
                if is_visible(*new_edge, obstacles):

                    node_parent, new_node = new_edge
                    parentidx = self.get_id(node_parent)
                    newidx = self.add_vex(new_node)

                    return euclidian_distance(new_node, node_parent), newidx, parentidx


def _sample_uniform(board_shape):
    deltaX, deltaY = board_shape
    pt = np.random.rand(2) - 0.5
    pt[0] *= 2 * deltaX
    pt[1] *= 2 * deltaY
    return pt


def _sample_ellipse(startpos, endpos, c_best):
    """Sample uniformly from the 2-D prolate hyperellipsoid (Gammell et al. 2014)."""
    sp = np.asarray(startpos, dtype=float)
    ep = np.asarray(endpos, dtype=float)
    c_min = float(np.linalg.norm(ep - sp))
    if c_best <= c_min:
        return None
    a = c_best / 2.0
    b = math.sqrt(max(c_best ** 2 - c_min ** 2, 0.0)) / 2.0
    r = math.sqrt(random())
    theta_s = 2 * math.pi * random()
    local = np.array([a * r * math.cos(theta_s), b * r * math.sin(theta_s)])
    angle = math.atan2(ep[1] - sp[1], ep[0] - sp[0])
    ca, sa = math.cos(angle), math.sin(angle)
    R = np.array([[ca, -sa], [sa, ca]])
    center = (sp + ep) / 2.0
    return center + R @ local


def _connect_from_point(G, pt, obstacles, radius):
    """Find the best visible parent for pt. Returns (edge_dist, newidx, parentidx) or None."""
    candidates = []
    for node in G.vertices:
        d = euclidian_distance(node, pt)
        if d <= radius:
            idx = G.get_id(node)
            candidates.append((d + G.distances[idx], node, pt))
    for _, node_parent, new_node in sorted(candidates, key=lambda x: x[0]):
        if is_visible(node_parent, new_node, obstacles):
            parentidx = G.get_id(node_parent)
            newidx = G.add_vex(new_node)
            return euclidian_distance(new_node, node_parent), newidx, parentidx
    return None


def RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length, n_iter=2*10**3, time_for_ending=200, G=None, board_shape=None):
    ''' RRT star algorithm '''
    if board_shape is None:
        board_shape = BOARD_SHAPE

    endpos = T[:2]

    if G == None:
        G = Graph(startpos, endpos)

    gobs_4planning = [[v[:2] for v in oi if v[-1]==0] for oi in ground_obs]

    feasible_dist = math.sqrt(TETHER_LENGTH**2 - (T[-1] - MARSUPIAL_HEIGHT + UAV_RADIUS)**2)
    init_t = time.time()
    G._init_time = init_t
    iter = 0
    # while iter < n_iter and time.time() - init_t < time_for_ending:
    while True:
        if time.time() - init_t >= time_for_ending:
            break

        d, newidx, parentidx = G.randomPosition(board_shape[:2], gobs_4planning, radius)

        G.add_edge(newidx, parentidx, d)
        G.distances[newidx] = G.distances[parentidx] + d
        newvex = G.vertices[newidx]

        ####
        if euclidian_distance(newvex, G.endpos) <= feasible_dist:

            top = [*newvex, MARSUPIAL_HEIGHT - UAV_RADIUS]
            cat, length, t = get_min_catenary_rectangles(
                np.array(top), T,
                ground_obs + aerial_obs,
                euclidian_distance_lists(top, G.endpos),
                TETHER_LENGTH, k_length, col2=True)

            if length>0:

                endidx = G.add_vex(G.endpos)
                G.success = True
                G.add_edge(newidx, endidx, length)

                if len(G.opt_cat) == 0 or G.distances[newidx]+length < G.distances[endidx]:
                    G.distances[endidx] = G.distances[newidx]+length
                    G.opt_cat = cat
                    G.opt_length = length
                    G._improve_log.append((time.time() - init_t, G.distances[endidx], length))

                if G.distances[endidx] < 78 or time.time() - init_t >= time_for_ending:
                    break

        # dist = euclidian_distance(newvex, G.endpos)
        # if dist < 2 * radius:
        #     endidx = G.add_vex(G.endpos)
        #     G.add_edge(newidx, endidx, dist)
        #     try:
        #         G.distances[endidx] = min(G.distances[endidx], G.distances[newidx]+dist)
        #     except:
        #         G.distances[endidx] = G.distances[newidx]+dist

        iter += 1

    print("total time", time.time()-init_t)

    return G


def dijkstra(G, endnode):
    '''
    Dijkstra algorithm for finding shortest path from start position to end.
    '''
    srcIdx = G.vex2idx[tuple(G.startpos)]
    endnode = G.endpos
    endidx = G.vex2idx[tuple(endnode)]
    dist = G.distances[endidx]

    # build dijkstra
    nodes = list(G.neighbors.keys())
    dist = {node: float('inf') for node in nodes}
    prev = {node: None for node in nodes}
    dist[srcIdx] = 0

    while nodes:
        curNode = min(nodes, key=lambda node: dist[node])
        nodes.remove(curNode)
        if dist[curNode] == float('inf'):
            break

        for neighbor, cost in G.neighbors[curNode]:
            newCost = dist[curNode] + cost
            if newCost < dist[neighbor]:
                dist[neighbor] = newCost
                prev[neighbor] = curNode

    # retrieve path
    path = deque()
    curNode = endidx
    while prev[curNode] is not None:
        path.appendleft(G.vertices[curNode])
        curNode = prev[curNode]
    path.appendleft(G.vertices[curNode])
    length = G.distances[prev[endidx]]
    return list(path)[:-1], length


def Informed_RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length,
                      time_for_ending=60, board_shape=None):
    """Informed RRT*: once a solution exists, restricts sampling to the prolate
    hyperellipsoid defined by (startpos, T[:2], c_best), ignoring regions that
    cannot improve the current best path (Gammell et al., IROS 2014)."""
    if board_shape is None:
        board_shape = BOARD_SHAPE
    endpos = T[:2]
    G = Graph(startpos, endpos)
    gobs_4planning = [[v[:2] for v in oi if v[-1] == 0] for oi in ground_obs]
    feasible_dist = math.sqrt(TETHER_LENGTH ** 2 - (T[-1] - MARSUPIAL_HEIGHT + UAV_RADIUS) ** 2)
    init_t = time.time()
    G._init_time = init_t

    while True:
        if time.time() - init_t >= time_for_ending:
            break
        if G.success:
            endidx = G.vex2idx[tuple(G.endpos)]
            c_best = G.distances[endidx]
            pt = _sample_ellipse(tuple(startpos), tuple(endpos), c_best)
            if pt is None:
                pt = _sample_uniform(board_shape[:2])
        else:
            pt = _sample_uniform(board_shape[:2])

        result = _connect_from_point(G, pt, gobs_4planning, radius)
        if result is None:
            continue

        d, newidx, parentidx = result
        G.add_edge(newidx, parentidx, d)
        G.distances[newidx] = G.distances[parentidx] + d
        newvex = G.vertices[newidx]

        if euclidian_distance(newvex, G.endpos) <= feasible_dist:
            top = [*newvex, MARSUPIAL_HEIGHT - UAV_RADIUS]
            cat, length, t = get_min_catenary_rectangles(
                np.array(top), T,
                ground_obs + aerial_obs,
                euclidian_distance_lists(top, G.endpos),
                TETHER_LENGTH, k_length, col2=True)

            if length > 0:
                endidx = G.add_vex(G.endpos)
                G.success = True
                G.add_edge(newidx, endidx, length)

                if len(G.opt_cat) == 0 or G.distances[newidx] + length < G.distances[endidx]:
                    G.distances[endidx] = G.distances[newidx] + length
                    G.opt_cat = cat
                    G.opt_length = length
                    G._improve_log.append((time.time() - init_t, G.distances[endidx], length))

                if time.time() - init_t >= time_for_ending:
                    break

    print("total time", time.time() - init_t)
    return G


def Smart_RRT_star(startpos, T, ground_obs, aerial_obs, radius, k_length,
                   time_for_ending=60, board_shape=None, p_smart=0.3):
    """Smart RRT* (RRT*-Smart, Nasir et al. 2013): path-biased sampling near
    the current best ground-path waypoints after a solution is found, causing
    the tree to refine the path from within rather than exploring blindly."""
    if board_shape is None:
        board_shape = BOARD_SHAPE
    endpos = T[:2]
    G = Graph(startpos, endpos)
    gobs_4planning = [[v[:2] for v in oi if v[-1] == 0] for oi in ground_obs]
    feasible_dist = math.sqrt(TETHER_LENGTH ** 2 - (T[-1] - MARSUPIAL_HEIGHT + UAV_RADIUS) ** 2)
    init_t = time.time()
    G._init_time = init_t
    best_path = []
    sigma = 8.0

    while True:
        if time.time() - init_t >= time_for_ending:
            break
        if G.success and best_path and random() < p_smart:
            pivot = np.array(best_path[int(random() * len(best_path))][:2], dtype=float)
            pt = pivot + np.random.randn(2) * sigma
        else:
            pt = _sample_uniform(board_shape[:2])

        result = _connect_from_point(G, pt, gobs_4planning, radius)
        if result is None:
            continue

        d, newidx, parentidx = result
        G.add_edge(newidx, parentidx, d)
        G.distances[newidx] = G.distances[parentidx] + d
        newvex = G.vertices[newidx]

        if euclidian_distance(newvex, G.endpos) <= feasible_dist:
            top = [*newvex, MARSUPIAL_HEIGHT - UAV_RADIUS]
            cat, length, t = get_min_catenary_rectangles(
                np.array(top), T,
                ground_obs + aerial_obs,
                euclidian_distance_lists(top, G.endpos),
                TETHER_LENGTH, k_length, col2=True)

            if length > 0:
                endidx = G.add_vex(G.endpos)
                G.success = True
                G.add_edge(newidx, endidx, length)
                new_cost = G.distances[newidx] + length

                if len(G.opt_cat) == 0 or new_cost < G.distances[endidx]:
                    G.distances[endidx] = new_cost
                    G.opt_cat = cat
                    G.opt_length = length
                    G._improve_log.append((time.time() - init_t, new_cost, length))
                    best_path, _ = dijkstra(G, T)

                if time.time() - init_t >= time_for_ending:
                    break

    print("total time", time.time() - init_t)
    return G


def plot(G, obstacles, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''

    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.startpos[0], G.startpos[1], c='black')
    ax.scatter(G.endpos[0], G.endpos[1], c='black')

    if obstacles:
        from scipy.spatial import ConvexHull
        for o in obstacles:
            points = np.array([np.array(p[:2]) for p in o if p[-1]==0])
            h = ConvexHull(points)
            for simplex in h.simplices:
                ax.plot(points[simplex, 0], points[simplex, 1], 'k-')


    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)

    plt.show()


def pathSearch(startpos, endpos, obstacles, n_iter, radius, stepSize):
    G = RRT_star(startpos, endpos, obstacles, n_iter, radius, stepSize)
    if G.success:
        path = dijkstra(G)
        # plot(G, obstacles, radius, path)
        return path


def rrt_sequential(plotting=False):

    path = "scenarios/S3.pkl"

    with open(path, "rb") as f:
        s = pkl.loads(f.read())

    S = s["S"]
    Ts = s["T"]
    ground_obs = s["ground_obstacles"]
    aerial_obs = s["aerial_obstacles"]
    k_length=26
    radius = 10**6

    tt = 0
    gpaths = []
    opt_tops = []
    total_length = 0
    for i, T in enumerate(Ts):

        t = time.time()
        G = RRT_star(S[:2],T,ground_obs,aerial_obs, radius, k_length, n_iter=10**6, time_for_ending=20)
        tt += time.time() - t

        ground_path, gpath_length = dijkstra(G, T)
        # print("gpath_length", gpath_length)
        if plotting:
            plot(G, ground_obs, ground_path)

        total_length += gpath_length
        gpaths.append(ground_path)
        optX, cat, apath_length = ground_path[-1], G.opt_cat, G.opt_length
        topX = tuple([*optX[:2], MARSUPIAL_HEIGHT-UAV_RADIUS])
        opt_ctop = {topX: {"length":apath_length, "tether":cat}}
        opt_tops.append(opt_ctop)

        S = tuple([*optX[:2], 0])
        total_length += apath_length
        # print("apath_length", apath_length)

        if i < len(Ts)-1:
            total_length += apath_length
            # print("comeback apath_length", apath_length)


    print(total_length, tt)

    d = {
        "total_length": total_length,
        "total_time": tt,
        "scenario": "S3",
        "S": S,
        "Ts": Ts,
        "ground_obs": ground_obs,
        "aerial_obs": aerial_obs,
        "gpaths": gpaths,
        "opt_tops": opt_tops
    }

    with open("scenarios/S3_rrt.pkl", "wb") as f:
        f.write(pkl.dumps(d))


def example():

    # # Best path resutls:
    # time for graph 20.010276794433594
    # total_path length 83.3471842895182

    with open("scenarios/S2.pkl", "rb") as f:
        scenario = pkl.load(f)

    S = scenario["S"]
    T = scenario["T"]
    a_obs, g_obs = scenario["aerial_obstacles"], scenario["ground_obstacles"]
    radius = 10**6 # RRT parameter
    k_length = 26

    tt = time.time()
    G = RRT_star(S[:2], T, g_obs, a_obs, radius, k_length, n_iter=10**6, time_for_ending=20)
    # print("time for graph", time.time()-tt)

    if G.success:
        ground_path, gpath_length = dijkstra(G, T)
        plot(G, g_obs, ground_path)

        optX, cat, apath_length = ground_path[-1], G.opt_cat, G.opt_length
        topX = tuple([*optX[:2], MARSUPIAL_HEIGHT-UAV_RADIUS])
        opt_ctop = {topX: {"length":apath_length, "tether":cat}}
        rrt_sol = {
            "S": S,
            "T": T,
            "scenario": "S1",
            "ground_path": ground_path,
            "opt_ctop": opt_ctop
        }

        with open("scenarios/S1_rrt.pkl", "wb") as f:
            f.write(pkl.dumps(rrt_sol))

        print("total_path length", gpath_length + apath_length)

    else:
        print("Not SUCCEDED")
        plot(G, g_obs, radius)



def run_benchmark(n_runs=30):
    # n_runs: number of independent runs per (scenario, algorithm) pair.
    # Increase for statistically robust results (paper uses 30); set to 1 for a quick check.
    import os, contextlib

    SCENARIOS        = ["S2", "S3", "S5"]
    # SCENARIOS        = ["S3"]
    TIME_CHECKPOINTS = [10, 20]
    N_RUNS           = n_runs
    TOTAL_TIME       = 20   # seconds per (scenario, algorithm, run)
    k_length         = 26
    radius           = 10**6

    # ------------------------------------------------------------------ helpers
    def best_cost_at(log, t):
        """Best cumulative cost recorded at or before time t (None = unsolved)."""
        best = None
        for (elapsed, cost, _) in log:
            if elapsed <= t:
                best = cost
        return best

    def best_entry_at(log, t):
        """Return (cost, aerial) of the best solution at or before time t, or None."""
        best = None
        for (elapsed, cost, aerial) in log:
            if elapsed <= t:
                if best is None or cost < best[0]:
                    best = (cost, aerial)
        return best

    def _call(algo_key, startpos, T, ground_obs, aerial_obs, time_budget, board_shape):
        if algo_key == "rrt":
            G = RRT_star(startpos[:2], T, ground_obs, aerial_obs,
                         radius, k_length, n_iter=10**6, time_for_ending=time_budget,
                         board_shape=board_shape)
        elif algo_key == "informed":
            G = Informed_RRT_star(startpos[:2], T, ground_obs, aerial_obs,
                                  radius, k_length, time_for_ending=time_budget,
                                  board_shape=board_shape)
        else:
            G = Smart_RRT_star(startpos[:2], T, ground_obs, aerial_obs,
                               radius, k_length, time_for_ending=time_budget,
                               board_shape=board_shape)
        return G

    def run_single(algo_key, data, time_budget, board_shape):
        with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
            G = _call(algo_key, data["S"], data["T"],
                      data["ground_obstacles"], data["aerial_obstacles"],
                      time_budget, board_shape)
        return G._improve_log

    def run_sequential(algo_key, data, time_per_target, board_shape):
        """Run two sequential targets; returns a global improve_log with cumulative costs."""
        targets    = data["T"]
        ground_obs = data["ground_obstacles"]
        aerial_obs = data["aerial_obstacles"]
        S_cur      = data["S"]
        global_log = []
        time_offset = 0.0
        cost_offset = 0.0

        for i, T in enumerate(targets):
            is_last = (i == len(targets) - 1)
            with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
                G = _call(algo_key, S_cur, T, ground_obs, aerial_obs, time_per_target, board_shape)

            # Return trip only for non-last targets (drone comes back to UGV)
            for (lt, lc, ll) in G._improve_log:
                return_leg = 0 if is_last else ll
                global_log.append((time_offset + lt, cost_offset + lc + return_leg, ll))

            if not G.success:
                break   # subsequent targets cannot be reached

            entry = best_entry_at(G._improve_log, time_per_target)
            if entry is None:
                break
            return_leg = 0 if is_last else entry[1]
            cost_offset += entry[0] + return_leg
            time_offset += time_per_target

            # Advance UGV start to the tether-anchor ground position
            ground_path, _ = dijkstra(G, T)
            optX  = ground_path[-1]
            S_cur = tuple([*optX[:2], 0])

        return global_log

    # --------------------------------------------------------------- main loop
    results = {}   # (scenario, algo_name, checkpoint) -> list of costs

    algo_defs = [
        ("RRT*",          "rrt"),
        ("Informed RRT*", "informed"),
        ("Smart RRT*",    "smart"),
    ]

    for sname in SCENARIOS:
        with open(f"scenarios/{sname}.pkl", "rb") as f:
            data = pkl.load(f)

        is_sequential = isinstance(data.get("T"), list)
        n_targets     = len(data["T"]) if is_sequential else 1
        time_per_t    = TOTAL_TIME // n_targets   # 40s single, 20s each for S3
        bshape        = scenario_board_shape(data)

        for aname, akey in algo_defs:
            print(f"[{sname}] {aname} — {N_RUNS} runs × {TOTAL_TIME}s "
                  f"({'20s×2 targets' if is_sequential else '40s'})"
                  f"  board=({bshape[0]:.0f},{bshape[1]:.0f})", flush=True)

            run_costs = {t: [] for t in TIME_CHECKPOINTS}

            for run_i in range(N_RUNS):
                print(f"  run {run_i+1:2d}/{N_RUNS}", end="\r", flush=True)
                if is_sequential:
                    log = run_sequential(akey, data, time_per_t, bshape)
                else:
                    log = run_single(akey, data, TOTAL_TIME, bshape)

                for t in TIME_CHECKPOINTS:
                    run_costs[t].append(best_cost_at(log, t))

            for t in TIME_CHECKPOINTS:
                results[(sname, aname, t)] = run_costs[t]
            print()   # newline after \r progress

    # ------------------------------------------------------------------- table
    COL = 22

    def fmt(costs):
        valid = [c for c in costs if c is not None]
        sr = 100 * len(valid) / N_RUNS
        if not valid:
            return f"{'N/A (0%)':>{COL}}"
        return f"{np.mean(valid):6.2f} ± {np.std(valid):5.2f}  ({sr:.0f}%)"

    hdr = f"{'':6}  {'Algorithm':<16}" + "".join(f"  {'@'+str(t)+'s':>{COL}}" for t in TIME_CHECKPOINTS)
    bar = "-" * len(hdr)

    print(f"\n{'='*len(bar)}")
    print(f"BENCHMARK  —  mean path cost ± std  (success rate over {N_RUNS} runs)")
    print(f"             lower cost = better path;  S3 budget = 30 s per target")
    print(f"{'='*len(bar)}")
    print(hdr)

    for sname in SCENARIOS:
        print(bar)
        for aname, _ in algo_defs:
            row = f"{sname:<6}  {aname:<16}"
            for t in TIME_CHECKPOINTS:
                row += f"  {fmt(results[(sname, aname, t)])}"
            print(row)

    print(f"{'='*len(bar)}")


if __name__ == '__main__':

    run_benchmark()

    # example()
    # rrt_sequential(plotting=False)


