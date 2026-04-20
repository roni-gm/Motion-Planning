# D* (Dynamic A-Star) Path Planning Algorithm

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![Library](https://img.shields.io/badge/Library-matplotlib%20%7C%20heapq%20%7C%20math-orange)
![Type](https://img.shields.io/badge/Type-Grid--Based%20%7C%20Dynamic%20Replanning-red)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

*A dynamic replanning algorithm that efficiently updates shortest paths when the environment changes — without replanning from scratch.*

</div>

---

## Table of Contents
- [Overview](#overview)
- [How It Works](#how-it-works)
- [Algorithm Details](#algorithm-details)
- [Code Structure](#code-structure)
- [Parameters](#parameters)
- [How to Run](#how-to-run)
- [Complexity Analysis](#complexity-analysis)
- [D* vs D* Lite vs LPA*](#d-vs-d-lite-vs-lpa)
- [Results & Visualization](#results--visualization)
- [Future Development](#future-development)

---

## Overview

**D\*** (Dynamic A-Star), originally introduced by Anthony Stentz in 1994, and later refined into **D\* Lite** by Sven Koenig and Maxim Likhachev (2002), is a family of **incremental heuristic search algorithms** designed specifically for **dynamic environments** — where obstacles may appear or disappear as the robot moves.

While A* must replan from scratch every time the map changes, D\* Lite can **repair the existing solution** by only updating the parts of the search that are affected by the change. This makes it dramatically more efficient in partially-known or changing environments.

This implementation follows the **D\* Lite** variant using `rhs-values` and `g-values` with a priority queue. The planner searches **backward from goal to start**, so when a change is detected near the robot, the affected portion of the graph is recomputed locally without touching the rest of the path.

### Key Characteristics
| Property | Value |
|---|---|
| **Completeness** | Yes — finds a path if one exists |
| **Optimality** | Yes — optimal for current known map |
| **Search Direction** | Backward (goal → start) |
| **Replanning** | Incremental — only affected nodes are updated |
| **Environment** | Dynamic 2D grid map |
| **Motion Model** | 8-directional movement |

---

## How It Works

D\* Lite maintains two cost estimates for each node `u`:

| Value | Meaning |
|---|---|
| `g(u)` | Current best-known cost from `u` to goal |
| `rhs(u)` | One-step lookahead value: min cost achievable in one move from `u`'s successors |

A node is considered **consistent** when `g(u) == rhs(u)`. An **inconsistent** node is placed in the priority queue to be resolved.

### Priority Key Function
Each node in the priority queue is ordered by a two-component key:

```
key(u) = (min(g(u), rhs(u)) + h(start, u),  min(g(u), rhs(u)))
```

Where `h(start, u)` is the Euclidean heuristic from `start` to `u`.

### Step-by-Step Process

```
INITIALIZATION:
  - Set rhs(goal) = 0, g(all) = INF
  - Push goal into priority queue

COMPUTE SHORTEST PATH:
  While priority queue not empty:
    Pop node u with smallest key
    If g(u) > rhs(u):       → node is underconsistent
      g(u) = rhs(u)
      Update all predecessors of u
    Else:                   → node is overconsistent
      g(u) = INF
      Update u and all predecessors

PATH RECONSTRUCTION:
  From start, greedily follow the neighbor with lowest (move_cost + g)
  until goal is reached

ON MAP CHANGE:
  For each changed edge:
    Update rhs values of affected nodes
    Push affected nodes back into queue
    Re-run ComputeShortestPath()
```

---

## Algorithm Details

### Backward Search Philosophy
D\* Lite computes the shortest path **from goal to start** (backward). This is key for efficiency: when the robot moves forward and discovers new obstacles ahead, the goal remains fixed — only the nodes near the robot's current position need to be re-evaluated.

### Two-Component Priority Key
The `(k1, k2)` key ensures correct ordering even during repairs:
```python
def key(u):
    g_u = g.get(u, INF)
    r_u = rhs.get(u, INF)
    k1 = min(g_u, r_u) + h(start, u)   # primary key
    k2 = min(g_u, r_u)                  # tiebreaker
    return (k1, k2)
```

### Lazy Deletion from Priority Queue
Rather than true deletion (which would be O(n)), this implementation uses **lazy deletion** via an `entry_finder` dictionary. Outdated entries are silently skipped when popped:

```python
def pq_top_key():
    while pq:
        k1, k2, _, u = pq[0]
        if entry_finder.get(u) == pq[0]:  # still valid?
            return (k1, k2)
        heapq.heappop(pq)  # discard stale entry
    return (INF, INF)
```

---

## Code Structure

```
dStar.py
├── class Dijkstra                          ← Shared base class (grid + motion model)
│   ├── __init__(ox, oy, resolution, robot_radius)
│   ├── class Node
│   ├── planning(sx, sy, gx, gy)           ← Standard Dijkstra planning
│   ├── calc_final_path(goal_node, closed_set)
│   ├── calc_obstacle_map(ox, oy)
│   ├── verify_node(node)
│   └── get_motion_model()
│
├── class AStar(Dijkstra)                   ← A* planner (inherits Dijkstra)
│   ├── planning(sx, sy, gx, gy)
│   └── calc_heuristic(n1, n2)
│
├── class DStar(Dijkstra)                   ← D* Lite planner (inherits Dijkstra)
│   └── planning(sx, sy, gx, gy)
│       ├── neighbors(u)                    ← Valid adjacent nodes
│       ├── h(u, v)                         ← Euclidean heuristic
│       ├── key(u)                          ← Two-component priority key
│       ├── pq_push / pq_remove / pq_top_key  ← Lazy-deletion heap ops
│       ├── update_vertex(u)                ← Recompute rhs and re-queue if needed
│       ├── compute_shortest_path()         ← Core D* Lite loop
│       └── Path reconstruction (greedy neighbor selection)
│
└── main()
      └── Builds map → runs DStar → plots result
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `sx, sy` | `-5.0, -5.0` | Start position (meters) |
| `gx, gy` | `50.0, 50.0` | Goal position (meters) |
| `grid_size` | `1.0` | Resolution of each grid cell (meters) |
| `robot_radius` | `1.0` | Physical radius of robot (meters) |
| `show_animation` | `True` | Toggle real-time visualization |

---

## How to Run

### Prerequisites
```bash
pip install matplotlib
```

### Execution
```bash
cd "Algoritma D Star"
python dStar.py
```

### Expected Output
A matplotlib window showing:
- **Black dots** → Obstacles / walls
- **Green circle** → Start position
- **Blue X** → Goal position
- **Red line** → Final optimal path computed by D\* Lite

---

## Complexity Analysis

| Metric | Value |
|---|---|
| **Initial Planning** | O((V + E) log V) — same as A* |
| **Replanning after k changes** | O(k · log V) — only affected nodes |
| **Space Complexity** | O(V) for g, rhs tables and priority queue |
| **Path Quality** | Globally optimal for current known map |

---

## D\* vs D\* Lite vs LPA*

| Feature | D* (Original) | D* Lite | LPA* |
|---|---|---|---|
| Search direction | Forward | Backward | Forward |
| Replanning | Yes | Yes | Yes |
| Complexity | Higher | Lower | Lower |
| Implementation | Complex | Moderate | Moderate |
| Basis | A* extension | LPA* + heuristic shift | Incremental Dijkstra |
| Recommended for | Historical | ✅ Robotics (current standard) | Static graph changes |

---

## Results & Visualization

The test environment features:
- Map boundary: x ∈ [-10, 60], y ∈ [-10, 60]
- Multiple internal dividing walls
- Start: (-5, -5) → Goal: (50, 50)

D\* Lite searches backward from the goal, propagating cost values through the grid. The robot then follows the cost gradient forward to reach the goal — and when obstacles change, only the locally affected portion of the cost map is repaired.

---

## Future Development

D\* Lite already handles dynamic environments, but there is substantial room for growth:

### 1. 🚨 Real-Time Sensor Integration
Connect to a **LiDAR or depth camera** (e.g., via ROS2 sensor topics) that feeds newly detected obstacles into the planner. When a new obstacle is detected, trigger `update_vertex` on affected cells and re-run `compute_shortest_path()` — all while the robot keeps moving.

### 2. 🧭 Full Navigation Stack with ROS2 Nav2
Register D\* Lite as a **custom global planner plugin** for ROS2 Nav2. Use `costmap_2d` as the live map input and publish the resulting path as a `nav_msgs/Path` message to the robot's controller.

### 3. 🌐 3D Voxel Map Replanning
Extend the grid from 2D to a **3D voxel grid** for drone navigation. D\* Lite's incremental nature makes it especially attractive here, as reconfiguring a 3D plan from scratch when one obstacle moves is computationally prohibitive.

### 4. 🔮 Predictive Obstacle Modeling
Enhance the planner with a **moving obstacle predictor** — using Kalman filters or LSTM networks to forecast where dynamic obstacles (humans, vehicles) will be in the next few seconds, allowing pre-emptive replanning before a collision risk occurs.

### 5. 📡 Multi-Robot Coordinated Replanning
In a multi-robot system, treat **other robots as dynamic obstacles**. Each robot runs its own D\* Lite instance, receives positions of all others, and continuously updates its plan — enabling decentralized, collision-aware coordination.

### 6. ⚖️ Weighted Cost Map with Semantic Labels
Integrate **semantic understanding** so the planner assigns traversal costs based on terrain type (e.g., grass = 2.0, concrete = 1.0, water = ∞). This is critical for outdoor robots operating in unstructured environments.

### 7. 🎮 Interactive Demo with Obstacle Drawing
Build a **GUI using `matplotlib` widgets** where the user can draw obstacles on the map in real time and watch D\* Lite replan instantly — a powerful demonstration of incremental search for academic presentations and lab reports.

---

## References

- Stentz, A. (1994). *Optimal and Efficient Path Planning for Partially-Known Environments*. ICRA 1994.
- Koenig, S., & Likhachev, M. (2002). *D\* Lite*. AAAI 2002.
- Koenig, S., & Likhachev, M. (2001). *Incremental A\**. NIPS 2001.
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Motion Planning Project</i>
</div>
