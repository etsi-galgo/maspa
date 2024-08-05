
# MArsupial Sequential Path-planning Approach (MASPA)

This repository contains an algorithm for sequential path planning of a marsupial robotics system, which consists of a UAV and a UGV connected by a tether. The algorithm minimizes the combined lengths of the paths traveled by the UAV and the UGV.

Details of the algorithm and the benchmark experiments can be seen in the publication:

"An efficient strategy for path planning with a tethered marsupial robotics system"

J. Capitán, J. M. Díaz-Báñez, M. A. Pérez-Cutiño, F. Rodríguez and I. Ventura

## Installation Guide and Usage

Requirements: Python 3.10

```
pip install -r requirements.txt
```

Two realistic scenarios with MASPA can be executed with:

```
python path_planning.py
```

## Main Files Description

The `path_planning.py` file is the main file of the repository. It is used to compute paths for different scenarios using MASPA with and without the visibility module. The main functions are:

- path_planning_smpp: Compute ground and aerial paths for the marsupial system using MASPA and the visibility module.

- path_planning_bf: MASPA without the visibility module.

- maspa_sequential: Use MASPA in a scenario with multiple sequential targets.

Use the `rrt_planning.py` file to compute the path planning baseline algorithm based on RRT*. The main functions are:

- RRT_star: Path planning for the marsupial system using a variant of the RRT* algorithm

- rrt_sequential: Path planning in a scenario with multiple sequential targets using the RRT*-based baseline algorithm.

Use the `constants.py` file to specify the height of the marsupial system on the ground, the radius of the UAV's height, the maximum length of the tether, the height of the take-off points, and the EPSILON value used to avoid numerical issues in the code.

## Other Files

Use `cat2.py` to compute collision-free catenaries.

Use `cvisibility.py` to obtain catenary visible points for the UAV to take off from atop the UGV.

Use `drawing.py` to plot the scenarios and paths that have been computed.

Use `generate_scenarios.py` to create both realistic and random scenarios.

Use `metrics.py` to compute path length metrics for individual scenarios and the mean path length for random scenarios.

The `tools.py` file contains auxiliary functions used in both the MASPA strategy and the RRT* baseline strategy.
