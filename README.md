
# **MArsupial Sequential Path-planning Approach (MASPA)** #

# MArsupial Sequential Path-planning Approach (MASPA)

This repository containts an algorithm for sequential path planning of a marsupial robotics system: composed by a UAV and a UGV connected by a tether. In the algorithm, the sum of the lengths of the paths of the UAV and the UAV is minimized. 

Details of the algorithm and the benchmark experiments can be seen in the publication:

An efficient strategy for path planning with a tethered marsupial robotics system. J. Capitán, J. M. Díaz-Báñez, M. A. Pérez-Cutiño, F. Rodríguez and I. Ventura.


# Installation Guide

## **Installation** ##

Requirements: Python 3.10

> pip install -r requirements.txt


# Usage

## **Usage** ##

A realistic scenario with maspa can be run with:

> python path_planning.py

# Files Description

## **Files Description** ##

## Main Files

The path_planning.py file is the main file of the repository. Is used to compute paths for different scenarios using maspa with and without the visibility module. The main functions are:

```
path_planning_smpp: Compute ground and aerial paths for the marsupial system with maspa and the visibility module
```

```
path_planning_bf: maspa without the visibility module
```

```
maspa_sequential: use maspa in an scenario with sequential targets
```

Use rrt_planning.py file for computing the path planning baseline algorithm based on RRT*. The main functions are: 

```
RRT_star: Path planning for the marsupial system using a variant of the RRT* algorithm
```

```
rrt_sequential: Path planning in an scenario with sequential targets using the RRT* baseline algorithm.
```

Use constants.py file to specify the height of the marsupial systems in the ground, the radius of the UAV heights, the masimum length of the tether, the height of the take-off points, and the EPSILON value used for avoiding numerical problems in the code.

## Other Files

Use cat2.py to compute collision-free catenaries.

Use cvisibility.py  to get catenary visible points for the UAV to take off from atop the UGV

Use drawing.py  for plotting the scenarios and paths computed 

Use generate_scenarios.py to build realistic and random scenarios

Use metrics.py to compute the path lengths metric for realistic scenarios and the mean in case of random scenarios.

The tools.py file contains the auxiliar functions used in the maspa strategy and the RRT* baseline strategy.


