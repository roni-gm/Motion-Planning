# Motion Planning Algorithms

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![License](https://img.shields.io/badge/License-MIT-green)
![Algorithms](https://img.shields.io/badge/Algorithms-5-orange)
![Status](https://img.shields.io/badge/Status-Active-brightgreen)

*A comprehensive collection of classical and sampling-based motion planning algorithms implemented in Python, with real-time visualization using Matplotlib.*

</div>

---

## Table of Contents
- [About This Project](#about-this-project)
- [Algorithms Implemented](#algorithms-implemented)
- [Repository Structure](#repository-structure)
- [Getting Started](#getting-started)
- [Algorithm Comparison](#algorithm-comparison)
- [References](#references)

---

## About This Project

This repository is a hands-on implementation of fundamental **motion planning algorithms** used in mobile robotics. Each algorithm is implemented from scratch in Python and tested on a 2D map environment, complete with real-time animation showing how each planner explores the space and finds a path from start to goal.

Motion planning is the problem of finding a collision-free path for a robot moving from a **start configuration** to a **goal configuration** in an environment with obstacles. It is a core capability of any autonomous robot — from warehouse AGVs to surgical robots to self-driving cars.

**Developed for:** Robotics Engineering — Motion Planning Course  
**Institution:** Politeknik Negeri Batam

---

## Algorithms Implemented

| # | Algorithm | Type | Optimal | Space |
|---|---|---|---|---|
| 1 | [A* (A-Star)](./Alogiritma%20A%20Star/README.md) | Grid-based, Informed | ✅ Yes | Discrete |
| 2 | [Dijkstra's](./Algoritma%20Dijkstra's/README.md) | Grid-based, Uninformed | ✅ Yes | Discrete |
| 3 | [D* (D-Star Lite)](./Algoritma%20D%20Star/README.md) | Grid-based, Dynamic | ✅ Yes | Discrete |
| 4 | [PRM](./Algoritma%20Probabilistic%20Road-Map%20(PRM)/README.md) | Sampling-based, Multi-query | ❌ Approx | Continuous |
| 5 | [RRT](./Algoritma%20Rapid-Exploring%20Random%20Trees%20(RRT)/README.md) | Sampling-based, Single-query | ❌ Sub-optimal | Continuous |

---

## Repository Structure

```
Motion-Planning/
│
├── README.md                                          ← This file
│
├── Alogiritma A Star/
│   ├── README.md                                      ← A* documentation
│   └── aStar.py                                       ← A* implementation
│
├── Algoritma Dijkstra's/
│   ├── README.md                                      ← Dijkstra documentation
│   └── dijkstra.py                                    ← Dijkstra implementation
│
├── Algoritma D Star/
│   ├── README.md                                      ← D* Lite documentation
│   └── dStar.py                                       ← D* Lite implementation
│
├── Algoritma Probabilistic Road-Map (PRM)/
│   ├── README.md                                      ← PRM documentation
│   ├── probabilistic_road_map_aStar.py                ← PRM + A* search
│   └── probabilistic_road_map_dijkstra.py             ← PRM + Dijkstra search
│
└── Algoritma Rapid-Exploring Random Trees (RRT)/
    ├── README.md                                      ← RRT documentation
    └── rapid_exploring_random_trees.py                ← RRT implementation
```

---

## Getting Started

### Prerequisites

All algorithms require **Python 3.8+** and the following libraries:

```bash
pip install matplotlib numpy scipy
```

### Running Each Algorithm

```bash
# A* Search
cd "Alogiritma A Star"
python aStar.py

# Dijkstra's
cd "Algoritma Dijkstra's"
python dijkstra.py

# D* Lite
cd "Algoritma D Star"
python dStar.py

# PRM with A* search
cd "Algoritma Probabilistic Road-Map (PRM)"
python probabilistic_road_map_aStar.py

# PRM with Dijkstra search
python probabilistic_road_map_dijkstra.py

# RRT
cd "Algoritma Rapid-Exploring Random Trees (RRT)"
python rapid_exploring_random_trees.py
```

---

## Algorithm Comparison

### When to Use Which Algorithm

| Scenario | Recommended Algorithm |
|---|---|
| Small grid map, need exact shortest path | **A\*** |
| Need full cost map from a single source | **Dijkstra's** |
| Environment changes dynamically during traversal | **D\* Lite** |
| High-dimensional space (robot arm), multiple queries | **PRM** |
| Need a fast feasible path, don't need optimality | **RRT** |
| Need fast + asymptotically optimal path | **RRT\*** *(extension)* |

### Performance Summary

| Algorithm | Completeness | Optimality | Speed | Memory | Dynamic |
|---|---|---|---|---|---|
| A* | ✅ Complete | ✅ Optimal | Fast | Medium | ❌ |
| Dijkstra | ✅ Complete | ✅ Optimal | Slower | Medium | ❌ |
| D* Lite | ✅ Complete | ✅ Optimal | Fast replan | Medium | ✅ |
| PRM | ✅ Prob. complete | ❌ Approx | Fast multi-query | Low | ❌ |
| RRT | ✅ Prob. complete | ❌ Sub-optimal | Very fast | Low | ❌ |

---

## References

- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE.
- Dijkstra, E. W. (1959). *A note on two problems in connexion with graphs*. Numerische Mathematik.
- Koenig, S., & Likhachev, M. (2002). *D\* Lite*. AAAI.
- Kavraki, L. E., et al. (1996). *Probabilistic roadmaps for path planning in high-dimensional configuration spaces*. IEEE Transactions on Robotics.
- LaValle, S. M. (1998). *Rapidly-Exploring Random Trees: A New Tool for Path Planning*. Tech Report.
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press. — [Free online](http://lavalle.pl/planning/)
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Motion Planning Project</i>
</div>
