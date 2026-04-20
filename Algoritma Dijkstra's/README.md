# Dijkstra's Path Planning Algorithm

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![Library](https://img.shields.io/badge/Library-matplotlib%20%7C%20heapq%20%7C%20math-orange)
![Type](https://img.shields.io/badge/Type-Grid--Based%20%7C%20Uninformed%20Search-blue)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

*The classic uniform-cost search algorithm that guarantees the globally optimal shortest path by systematically exploring every node by cost order.*

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
- [Results & Visualization](#results--visualization)
- [Future Development](#future-development)

---

## Overview

Dijkstra's algorithm, published by Edsger W. Dijkstra in 1959, is one of the most fundamental algorithms in computer science and robotics. It solves the **single-source shortest path problem** on a weighted graph with non-negative edge weights.

In a robotics context, the grid map is modeled as a weighted graph where each cell is a node and movement costs (cardinal = 1.0, diagonal = √2) are the edge weights. Dijkstra's algorithm guarantees finding the **globally optimal path** but unlike A*, it has no directional bias — it explores all nodes uniformly by cost, making it the gold standard for **correctness** at the cost of **efficiency**.

This implementation also serves as the **base class** for both A\* and D\* algorithms in this project, demonstrating clean object-oriented inheritance for path planners.

### Key Characteristics
| Property | Value |
|---|---|
| **Completeness** | Yes — always finds a path if one exists |
| **Optimality** | Yes — guaranteed globally shortest path |
| **Search Strategy** | Uniform-cost (uninformed) |
| **Heuristic** | None |
| **Environment** | 2D discrete grid map |
| **Motion Model** | 8-directional movement |
| **Data Structure** | Binary min-heap (priority queue via `heapq`) |

---

## How It Works

Dijkstra's algorithm explores nodes in order of their **actual accumulated cost** from the start node. It maintains two sets:

- **Open Set** (Priority Queue): Nodes discovered but not yet finalized — sorted by cost `g(n)`
- **Closed Set** (Dictionary): Nodes whose shortest path from start has been confirmed

### Cost Function

$$f(n) = g(n)$$

Unlike A*, there is no heuristic component. The cost is purely the actual distance traveled from start:

| Direction | Movement | Cost |
|---|---|---|
| Cardinal (↑ ↓ ← →) | 1 grid cell | `1.0` |
| Diagonal (↗ ↘ ↙ ↖) | 1 grid cell | `√2 ≈ 1.414` |

### Step-by-Step Process

```
1. Initialize:
   - Push (cost=0, start_node) into priority queue
   - closed_set = {}

2. Loop until priority queue is empty:
   a. Pop node with lowest cost from priority queue
   b. If node already in closed_set → skip (stale entry)
   c. If current node == goal → PATH FOUND, reconstruct and return
   d. Add current node to closed_set
   e. For each neighbor of current:
      - Compute new_cost = current.cost + move_cost
      - If not obstacle and not out of bounds → push to priority queue

3. If queue is empty and goal not reached → NO PATH EXISTS
```

---

## Algorithm Details

### Priority Queue with Tie-Breaking
This implementation uses Python's `heapq` with an `itertools.count()` counter to break ties between nodes of equal cost. This prevents comparison errors between `Node` objects and ensures **FIFO ordering** for same-priority nodes:

```python
import heapq
import itertools

counter = itertools.count()
heapq.heappush(open_set, (cost, next(counter), node))
```

### Motion Model (8-Directional)
```python
@staticmethod
def get_motion_model():
    return [
        [1, 0, 1],        # Right
        [0, 1, 1],        # Up
        [-1, 0, 1],       # Left
        [0, -1, 1],       # Down
        [-1, -1, 1.414],  # Down-Left diagonal
        [-1, 1, 1.414],   # Up-Left diagonal
        [1, -1, 1.414],   # Down-Right diagonal
        [1, 1, 1.414],    # Up-Right diagonal
    ]
```

### Obstacle Inflation
Each obstacle is inflated by `robot_radius` during map construction, ensuring the planned path keeps the robot's body clear of all obstacles.

---

## Code Structure

```
dijkstra.py
├── class Dijkstra                         ← Base planner class
│   ├── __init__(ox, oy, resolution, robot_radius)
│   │     └── Builds inflated obstacle map
│   ├── class Node
│   │     └── Stores (x, y, cost, parent_index)
│   ├── planning(sx, sy, gx, gy)
│   │     └── Main Dijkstra loop with heapq priority queue
│   ├── calc_final_path(goal_node, closed_set)
│   │     └── Reconstructs path by backtracking parent pointers
│   ├── calc_obstacle_map(ox, oy)
│   │     └── Builds boolean 2D inflated obstacle grid
│   ├── verify_node(node)
│   │     └── Checks bounds and obstacle collision
│   └── get_motion_model()
│         └── Returns 8-direction motion vectors
│
├── class AStar(Dijkstra)                  ← Inherits Dijkstra, overrides planning()
│   ├── planning(sx, sy, gx, gy)
│   │     └── A* with f = g + h priority
│   └── calc_heuristic(n1, n2)
│         └── Euclidean distance heuristic
│
└── main()
      └── Builds map → runs Dijkstra → plots result
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
cd "Algoritma Dijkstra's"
python dijkstra.py
```

### Expected Output
A matplotlib window showing:
- **Black dots** → Obstacles / walls
- **Green circle** → Start position
- **Blue X** → Goal position
- **Red line** → Final optimal path

---

## Complexity Analysis

| Metric | Value |
|---|---|
| **Time Complexity** | O((V + E) log V) with binary heap |
| **Space Complexity** | O(V) for priority queue and closed set |
| **Nodes Explored** | All reachable nodes (no heuristic guidance) |
| **Path Quality** | Globally optimal |

### Performance Comparison

| Feature | Dijkstra | A* |
|---|---|---|
| Heuristic | ❌ None | ✅ Euclidean |
| Nodes explored | More (exhaustive) | Fewer (guided) |
| Optimal path | ✅ Yes | ✅ Yes |
| Speed | Slower | Faster |
| Use case | Unknown goal direction | Known approximate goal direction |

> **When to use Dijkstra over A\*:** When multiple goals need to be evaluated simultaneously, or when computing the full cost map from a single source is required (e.g., coverage planning, Voronoi-based planning).

---

## Results & Visualization

The test environment features:
- Map boundary: x ∈ [-10, 60], y ∈ [-10, 60]
- Multiple internal walls creating a maze-like structure
- Start: (-5, -5) → Goal: (50, 50)

Dijkstra expands nodes radially outward from the start, systematically computing the true shortest distance to every reachable cell before reaching the goal.

---

## Future Development

This implementation is a strong base for further development. Here are potential directions to explore:

### 1. 🗂️ Full Cost-Map Generation (Wavefront Planner)
Run Dijkstra from a **fixed source** without a termination goal to produce a **complete cost map**. Any robot in the environment can then follow the gradient of this map to reach the source — useful for multi-robot systems and coverage tasks.

### 2. ⚡ Bidirectional Dijkstra
Run **two simultaneous Dijkstra searches** — one forward from start, one backward from goal — that terminate when their frontiers meet. This can reduce the number of explored nodes by roughly half on symmetric graphs.

### 3. 🗺️ Occupancy Grid Integration with ROS2
Replace the manual obstacle list with a **ROS2 `nav_msgs/OccupancyGrid`** topic so the planner can consume real-time sensor data from LiDAR or SLAM systems, enabling deployment on actual robots.

### 4. 📐 Weighted Terrain / Cost Map
Extend the grid to support **non-uniform traversal costs** (e.g., rough terrain = higher cost, smooth pavement = lower cost). This turns Dijkstra into a full terrain-aware planner — essential for outdoor UGV (Unmanned Ground Vehicle) navigation.

### 5. 🔀 Theta* Extension
Implement **Theta\*** on top of Dijkstra to allow **any-angle paths** — instead of restricting movement to 8 discrete directions, Theta\* uses line-of-sight checks to connect nodes directly, producing smoother and shorter paths.

### 6. 🤖 Multi-Robot Pathfinding
Scale up with **Conflict-Based Search (CBS)** which runs individual Dijkstra searches for each robot and resolves spatial-temporal conflicts, enabling collision-free coordination of multiple robots on shared maps.

### 7. 📊 Performance Profiling Tool
Add an optional profiling mode that records **nodes expanded, time elapsed, and path length** for each run, then generates a comparison report against A* and other planners on identical maps.

---

## References

- Dijkstra, E. W. (1959). *A note on two problems in connexion with graphs*. Numerische Mathematik, 1(1), 269–271.
- Cormen, T. H., et al. (2009). *Introduction to Algorithms* (3rd ed.). MIT Press.
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Motion Planning Project</i>
</div>
