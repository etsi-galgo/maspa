# MArsupial Sequential Path-planning Approach (MASPA)

Code for the paper:

> **"MASPA: An efficient strategy for path planning with a tethered marsupial robotics system"**  
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
├── maspa_planning.py     # MASPA algorithm entry point (SMPP and BF variants)
├── run_benchmark.py      # RRT* baseline algorithms (RRT*, Informed RRT*, Smart RRT*)
├── plot_comparison.py    # 3-D comparison plots — MASPA vs all RRT variants
├── path_lengths.py       # Print path-length table from cached plot data
├── time_to_beat_maspa.py # Measure how long each RRT variant takes to beat MASPA
├── metrics.py            # Metrics over random experiments
├── generate_scenarios.py # Scenario construction and serialisation
├── drawing.py            # 3-D/2-D visualisation utilities
├── cvisibility.py        # Catenary-visibility module (candidate take-off points)
├── cat2.py               # Catenary computation and collision checking
├── tools.py              # Shared geometry utilities (Dijkstra, visibility graph, …)
├── planners.py           # pyvisgraph-based Dijkstra planner
├── constants.py          # Physical parameters (tether length, robot height, …)
├── requirements.txt
├── reproduce_all.py      # One-shot script: runs everything end-to-end
└── scenarios/            # Serialised scenarios and cached results (.pkl)
    ├── S2.pkl                      # Scenario 2 — building + chimney (single target)
    ├── S3.pkl                      # Scenario 3 — building + balconies (sequential)
    ├── S5.pkl                      # Scenario 5 — indoor crowded (single target)
    ├── S1_maspa.pkl                # Saved MASPA solution on S2
    ├── S1_maspabf.pkl              # Saved MASPA-BF solution on S2
    ├── S2_plot_data.pkl            # Cached comparison data for S2
    ├── S3_plot_data.pkl            # Cached comparison data for S3
    ├── S5_plot_data.pkl            # Cached comparison data for S5
    └── random_scenarios.pkl        # 1 000 pre-generated random instances
```

---

## Scenarios

| Scenario | Type | Description |
|----------|------|-------------|
| **S2** | Single target | Outdoor — building with a chimney. UGV navigates around the building; UAV must reach the chimney tip via a tethered catenary. |
| **S3** | Sequential (2 targets) | Outdoor — building with two balconies at different heights. MASPA is run sequentially for each target. |
| **S5** | Single target | Indoor crowded — obstacles occupy most of the space, leaving a narrow feasible region. |

---

## Algorithms

### MASPA — with catenary-visibility module

`path_planning_smpp` in `maspa_planning.py`

1. Divide the space around the target into `p` vertical planes.
2. In each plane, sample `q` candidate take-off points at the circle of radius `sqrt(L² − (h_T − h_UGV)²)`.
3. Filter candidates using the **catenary-visibility module** (`cvisibility.py`): retain only points from which a collision-free catenary exists to the target.
4. Run a multi-target variant of **Dijkstra** on the ground visibility graph to find the take-off point that minimises ground-path length + aerial-path length.

### MASPA-BF — brute force (no visibility module)

`path_planning_bf` in `maspa_planning.py`

Same as MASPA but skips the visibility filter: all `p × q` candidates are tested for a valid catenary. Produces identical optimal solutions — used to verify the visibility module's correctness.

### RRT* — randomised baseline

`RRT_star` in `run_benchmark.py`

Standard RRT* adapted to the marsupial problem. The tree grows in the 2-D ground plane; when a node falls within tether reach of the target a catenary check is performed. Runs for a fixed time budget.

### Informed RRT* — ellipsoidal sampling

`Informed_RRT_star` in `run_benchmark.py`

Once a solution is found, sampling is restricted to the prolate hyperellipsoid defined by (start, target, current best cost), pruning regions that cannot improve the solution (Gammell et al., IROS 2014).

### Smart RRT* — path-biased sampling

`Smart_RRT_star` in `run_benchmark.py`

After a solution is found, a fraction `p_smart` of samples are drawn near the current best ground-path waypoints (Gaussian perturbation), causing the tree to refine from within rather than exploring blindly (Nasir et al., 2013).

---

## Reproducing All Results

### Quick start — run everything at once

```bash
python reproduce_all.py
```

This script executes all steps below in order, skipping scenario generation if the files already exist.

---

### Step-by-step

#### 1. (Optional) Regenerate scenario files

Pre-generated `.pkl` files are already committed. To regenerate from scratch:

```bash
python generate_scenarios.py
```

#### 2. Run MASPA on S2 (single target)

```bash
python maspa_planning.py
```

By default `__main__` calls `example()`, which runs both `path_planning_smpp` and `path_planning_bf` on `scenarios/S2.pkl` and writes:

- `scenarios/S1_maspa.pkl`
- `scenarios/S1_maspabf.pkl`

To run the sequential variant (S3), edit `maspa_planning.py` and uncomment `maspa_sequential()` in `__main__`.

#### 3. Run RRT baselines benchmark

```bash
python run_benchmark.py
```

Runs **RRT\***, **Informed RRT\***, and **Smart RRT\*** on all three scenarios (S2, S3, S5). Default settings:

- 30 runs per (scenario, algorithm) — adjustable via the `n_runs` parameter
- 20 s time budget per run
- Metrics reported at 10 s and 20 s checkpoints

Output: a formatted table printed to stdout with `mean ± std (success rate)`.

To run just once for a quick sanity check:

```python
from run_benchmark import run_benchmark
run_benchmark(n_runs=1)
```

`reproduce_all.py` uses `n_runs=1` by default. Change `N_RUNS = 1` near the top of that file to `N_RUNS = 30` to reproduce the full paper statistics.

#### 4. Generate 3-D comparison plots

```bash
python plot_comparison.py
```

Runs all four methods (MASPA + 3 RRT variants) on S2, S3, and S5 and saves:

```
scenarios/S2_comparison.png
scenarios/S3_comparison.png
scenarios/S5_comparison.png
```

Results are cached in `scenarios/<name>_plot_data.pkl` — delete the cache file to force a fresh run.

#### 5. Print path-length table

```bash
python path_lengths.py
```

Reads the `_plot_data.pkl` caches and prints a table of total path lengths (ground + tether, with the return-trip multiplier applied for sequential targets).

Requires step 4 to have been run first.

#### 6. Time to first beat MASPA

```bash
python time_to_beat_maspa.py
```

Runs each RRT variant once per scenario (no fixed budget, up to 600 s) and reports the exact second when it first produces a path shorter than MASPA, or "DNF" if it never does.

Requires step 4 (cache files).

#### 7. Compute metrics on random experiments

```bash
python metrics.py
```

Reads `scenarios/random_results_final.pkl` and reports mean ± std of ground-path length, aerial-path length, total path length, and computation time.

To regenerate random results, edit `maspa_planning.py` and call `run_random_experiments(1000)` from `__main__`. A new set of 1 000 random instances can be generated from `generate_scenarios.py`.

---

## Comparison of Approaches

### Single-target scenarios (S2, S5)

| Algorithm | Path cost formula |
|-----------|-------------------|
| MASPA / MASPA-BF | `d(S→X) + d(X→T)` — guaranteed optimal for the sampled candidate set |
| RRT* | probabilistic; quality improves with time budget |
| Informed RRT* | faster convergence than RRT* via ellipsoidal pruning |
| Smart RRT* | path-biased refinement after first solution |

### Sequential scenario (S3)

| Algorithm | Path cost formula |
|-----------|-------------------|
| MASPA | `d(S→X1) + 2·d(X1→T1) + d(X1→X2) + 2·d(X2→T2)` |
| RRT variants | same formula; each target solved independently with its own time budget |

Run `python path_lengths.py` (after `plot_comparison.py`) to see exact numbers.

### Key observations

- **Visibility module**: MASPA consistently outperforms MASPA-BF in computation time with identical solution quality, confirming the catenary-visibility filter is effective.
- **vs RRT variants**: MASPA produces shorter paths in significantly less time on all three scenarios. Informed and Smart RRT* improve over vanilla RRT* but rarely match MASPA within a 20 s budget.
- **Sequential extension**: solving each sub-problem optimally with MASPA gives a near-optimal sequential strategy.

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

Algorithm parameters `p` (number of vertical planes), `q` (samples per plane), and `k_length` (catenary length discretisation steps) are set per call in `maspa_planning.py` and `run_benchmark.py`. The paper uses `p=16, q=30, k_length=26`.
