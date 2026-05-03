# MArsupial Sequential Path-planning Approach (MASPA)

Code for the paper:

> **"An efficient strategy for path planning with a tethered marsupial robotics system"**  
> J. Capitán, J. M. Díaz-Báñez, M. A. Pérez-Cutiño, F. Rodríguez and I. Ventura

A marsupial robotic system consists of a UGV (ground robot) carrying a UAV (aerial robot) connected by a physical tether. MASPA computes paths for both robots jointly, minimising combined travel length while respecting the tether constraint.

---

## Table of Contents

1. [Installation](#installation)
2. [Repository Structure](#repository-structure)
3. [Scenarios](#scenarios)
4. [Algorithms](#algorithms)
5. [Reproducing All Results](#reproducing-all-results)
6. [Comparison of Approaches](#comparison-of-approaches)
7. [Configuration](#configuration)

---

## Installation

Requires **Python 3.10**.

```bash
git clone <repo-url>
cd maspa
pip install -r requirements.txt
```

Key dependencies: `numpy`, `matplotlib`, `scipy`, `pycatenary`, `pyvisgraph`, `trimesh`.

---

## Repository Structure

```
maspa/
├── path_planning.py      # MASPA algorithm (main entry point)
├── rrt_planning.py       # RRT* baseline algorithm
├── generate_scenarios.py # Scenario construction and serialisation
├── drawing.py            # 3D/2D visualisation and comparison plots
├── metrics.py            # Path-length and timing metrics over random experiments
├── cvisibility.py        # Catenary-visibility module (candidate take-off points)
├── cat2.py               # Catenary computation and collision checking
├── tools.py              # Shared geometry utilities (Dijkstra, visibility graph, …)
├── constants.py          # Physical parameters (tether length, robot height, …)
├── requirements.txt
└── scenarios/            # Serialised scenarios and algorithm results (.pkl)
    ├── S1.pkl            # Scenario 1 — column obstacles (single target)
    ├── S2.pkl            # Scenario 2 — building + chimney (single target)
    ├── S3.pkl            # Scenario 3 — building + balconies, revised (sequential)
    ├── S5.pkl            # Scenario 5 — indoors crawded
    ├── random_scenarios.pkl         # 1 000 random test instances
```

---

## Algorithms

### MASPA — with catenary-visibility module

`path_planning_smpp` in `path_planning.py`

1. Divide the space around the target into `p` vertical planes.
2. In each plane, sample `q` candidate take-off points at the circle of radius `sqrt(L² − (h_T − h_UGV)²)`.
3. Filter candidates using the **visibility module** (`cvisibility.py`): retain only points from which a collision-free catenary exists to the target.
4. Run a multi-target variant of **Dijkstra** on the ground visibility graph to find the take-off point minimising ground-path length + aerial-path length.

### MASPA-BF — brute force (no visibility module)

`path_planning_bf` in `path_planning.py`

Same as MASPA but skips the visibility filter: all `p × q` candidate points are tested for a valid catenary. Slower but produces identical optimal solutions — used to verify the visibility module's correctness.

### RRT* — randomised baseline

`RRT_star` in `rrt_planning.py`

A variant of the RRT* algorithm adapted to the marsupial problem. The graph is built in 2D (ground plane); when a new node falls within tether reach of the target, a catenary check is performed. The algorithm runs for a fixed time budget (`time_for_ending` seconds) or until a path shorter than a threshold is found.

---

## Reproducing All Results


### 1. Regenerate scenario files

```bash
python generate_scenarios.py
```

To regenerate S2, S3 or S5, uncomment the corresponding lines at the bottom of the file.

### 2. Run MASPA on the single-target scenario (S2)

```python
# In path_planning.py the S1() function runs S2.pkl
python path_planning.py
```

This runs `path_planning_smpp` selecting in the code the scenario `S2.pkl`, `S3.pkl`, or `S5.pkl`.

To run MASPA-BF instead, uncomment the `path_planning_bf` call and comment out `path_planning_smpp` inside the `S1()` function:
```python
# path_planning_smpp(...)   ← comment out
path_planning_bf(...)       # ← uncomment
```

### 3. Run RRT* on the single-target scenario (S2)

```python
# In rrt_planning.py the example() function runs S2.pkl
python rrt_planning.py
```

Writes `scenarios/S1_rrt.pkl`.

### 4. Run MASPA on the sequential scenario (S3 revised)

```python
python path_planning.py   # maspa_sequential() is called from __main__
```

Writes `scenarios/S33_maspa.pkl`.

To run on the original S3, replace `scenarios/S3.pkl` with `scenarios/S3_original.pkl` inside `maspa_sequential`.

### 5. Run RRT* on the sequential scenario (S3 revised)

```python
python rrt_planning.py    # rrt_sequential() is called from __main__
```

Writes `scenarios/S33_rrt.pkl`.

### 6. Plot comparison: MASPA vs RRT* (single-target, S2)

```python
python drawing.py   # calls plot_S1() then plot_S2() from __main__
```

`plot_S1()` overlays the MASPA path (solid) and the RRT* path (dashed) in the same 3D scene for `S2`. Images are saved to `images/`.

### 7. Plot comparison: MASPA vs RRT* (sequential, S3)

`plot_S2()` in `drawing.py` (also called from `__main__`) renders both algorithms on the sequential scenario.

### 8. Run random experiments

```python
# In path_planning.py, comment out S1() and maspa_sequential() and uncomment:
run_random_experiments(5, init=0)
```

Uses `scenarios/random_scenarios.pkl` (1 000 pre-generated instances) and writes results to `scenarios/random_results_16-30.pkl`.

To generate a new set of random instances:
```python
# In generate_scenarios.py, uncomment:
get_random_scenarios(1000, ground_n=10, aerial_n=15, block_thick=5,
                     board_size=(50,50,40), path="scenarios/random_scenarios.pkl")
```

### 9. Compute metrics on random experiments

```bash
python metrics.py
```

Reports mean and standard deviation of ground-path length, aerial-path length, total path length, and computation time over all random instances in `scenarios/random_results_16-30.pkl`.

---

## Comparison of Approaches

### Single-target scenario (S2 — building + chimney)

| Algorithm | Ground path | Aerial path | Total | Time (s) |
|-----------|------------|-------------|-------|----------|
| MASPA (p=16, q=30) | see `S1_maspa.pkl` | see `S1_maspa.pkl` | optimal | fast |
| MASPA-BF (p=16, q=30) | same as MASPA | same as MASPA | optimal | slower |
| RRT* | near-optimal | near-optimal | variable | 20 s budget |

MASPA and MASPA-BF always produce the same optimal solution; MASPA is faster because the visibility module prunes infeasible candidates before the catenary check. RRT* is probabilistic: solution quality varies between runs and is not guaranteed optimal.

### Sequential scenario (S3 revised — building + balconies)

| Algorithm | Total length | Time (s) |
|-----------|-------------|----------|
| MASPA | see `S33_maspa.pkl` | fast |
| RRT* | see `S33_rrt.pkl` | 2 × 20 s budget |

### Random experiments (p=16, q=30, 1 000 instances)

Run `python metrics.py` after completing step 8 above to reproduce the aggregated statistics reported in the paper.

### Key observations

- **Visibility module**: MASPA consistently outperforms MASPA-BF in computation time with identical solution quality, confirming that the catenary-visibility filter is effective at pruning the candidate set.
- **vs RRT***: MASPA produces shorter paths in significantly less time on structured scenarios. RRT* occasionally finds competitive paths but requires a large iteration budget and is sensitive to the random seed.
- **Sequential extension**: the UGV reuse pattern (the UGV drives to the take-off point, waits, then continues to the next target) makes MASPA's greedy-sequential strategy near-optimal because each sub-problem is solved optimally.

---

## Configuration

Edit `constants.py` to change physical parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MARSUPIAL_HEIGHT` | 3 | Height of the UGV platform (metres) |
| `UAV_RADIUS` | 1 | UAV clearance radius (metres) |
| `TETHER_LENGTH` | 50 | Maximum tether length (metres) |
| `EPSILON` | 1e-6 | Numerical tolerance |
| `MAX_ITERS` | 1e6 | Max iterations for random scenario generation |

Algorithm parameters `p` (number of vertical planes), `q` (samples per plane), and `k_length` (catenary length discretisation steps) are set per scenario in `path_planning.py` and `rrt_planning.py`. The paper uses `p=16, q=30, k_length=26`.
