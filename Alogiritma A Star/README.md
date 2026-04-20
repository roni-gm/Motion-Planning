# A* (A-Star) Path Planning Algorithm

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![Library](https://img.shields.io/badge/Library-matplotlib%20%7C%20math-orange)
![Type](https://img.shields.io/badge/Type-Grid--Based%20%7C%20Informed%20Search-green)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

*An informed heuristic search algorithm that finds the shortest path efficiently by combining actual cost and estimated cost to goal.*

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

A* (pronounced "A-star") is one of the most widely used path planning algorithms in robotics and AI. Unlike Dijkstra's algorithm which explores all directions equally, A* uses a **heuristic function** to guide its search toward the goal, making it significantly faster in most scenarios.

A* guarantees finding the **optimal (shortest) path** as long as the heuristic is **admissible** (never overestimates the true cost). This implementation uses **Euclidean distance** as the heuristic, which satisfies the admissibility condition for a 2D grid environment.

### Key Characteristics
| Property | Value |
|---|---|
| **Completeness** | Yes — always finds a path if one exists |
| **Optimality** | Yes — guaranteed shortest path (with admissible heuristic) |
| **Search Strategy** | Best-first (informed) |
| **Heuristic Used** | Euclidean distance |
| **Environment** | 2D discrete grid map |
| **Motion Model** | 8-directional movement |

---

## How It Works

A* evaluates each node using the evaluation function:

$$f(n) = g(n) + h(n)$$

| Term | Description |
|---|---|
| `f(n)` | Total estimated cost of the cheapest path through node `n` |
| `g(n)` | Actual cost from the start node to node `n` |
| `h(n)` | Heuristic estimated cost from node `n` to the goal |

The algorithm always selects the node with the **lowest f-value** from the open set, ensuring it always explores the most promising path first.

### Step-by-Step Process

```
1. Initialize:
   - Add start node to OPEN SET
   - Set g(start) = 0

2. Loop until OPEN SET is empty:
   a. Pick node with lowest f(n) = g(n) + h(n) from OPEN SET
   b. If current node == goal → PATH FOUND, reconstruct and return
   c. Move current node from OPEN SET → CLOSED SET
   d. For each neighbor of current node:
      - Skip if in CLOSED SET
      - Skip if obstacle or out of bounds
      - If not in OPEN SET → add it
      - If in OPEN SET with higher g-cost → update it

3. If OPEN SET is empty and goal not reached → NO PATH EXISTS
```

---

## Algorithm Details

### Heuristic Function
This implementation uses **Euclidean distance**:

```python
def calc_heuristic(self, goal_node, current_node):
    gx = self.calc_position(goal_node.x, self.min_x)
    gy = self.calc_position(goal_node.y, self.min_y)
    cx = self.calc_position(current_node.x, self.min_x)
    cy = self.calc_position(current_node.y, self.min_y)
    h = math.hypot(gx - cx, gy - cy)
    return h
```

### Motion Model (8-Directional)
The robot can move in 8 directions — 4 cardinal and 4 diagonal:

```
↖  ↑  ↗
←  ●  →     Cost: cardinal = 1.0, diagonal = √2 ≈ 1.414
↙  ↓  ↘
```

### Obstacle Inflation
The obstacle map inflates each obstacle by `robot_radius` to account for the physical size of the robot, preventing collisions at the boundary.

---

## Code Structure

```
aStar.py
├── class AStar
│   ├── __init__(ox, oy, resolution, robot_radius)
│   │     └── Initializes grid map and obstacle inflation
│   ├── class Node
│   │     └── Stores (x, y, g_cost, parent_index)
│   ├── planning(sx, sy, gx, gy)
│   │     └── Main A* search loop → returns path (rx, ry)
│   ├── calc_heuristic(goal_node, current_node)
│   │     └── Euclidean distance heuristic
│   ├── calc_final_path(goal_node, closed_set)
│   │     └── Backtrack through parent pointers to reconstruct path
│   ├── calc_obstacle_map(ox, oy)
│   │     └── Builds inflated 2D boolean obstacle grid
│   └── get_motion_model()
│         └── Returns 8-directional motion vectors
└── main()
      └── Defines map, obstacles, start/goal → runs planning → plots
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `sx, sy` | `-5.0, -5.0` | Start position (meters) |
| `gx, gy` | `50.0, 50.0` | Goal position (meters) |
| `grid_size` | `2.0` | Resolution of each grid cell (meters) |
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
cd "Alogiritma A Star"
python aStar.py
```

### Expected Output
```
min_x: -10
min_y: -10
max_x: 60
max_y: 60
x_width: 35
y_width: 35
Find goal
```

A matplotlib window will open showing:
- **Black dots** → Obstacles / walls
- **Green circle** → Start position
- **Blue X** → Goal position
- **Cyan X marks** → Nodes explored during search
- **Red line** → Final optimal path

---

## Complexity Analysis

| Metric | Value |
|---|---|
| **Time Complexity** | O(E log V) where E = edges, V = grid cells |
| **Space Complexity** | O(V) for open/closed sets and obstacle map |
| **Nodes Explored** | Significantly fewer than Dijkstra due to heuristic guidance |
| **Path Quality** | Globally optimal (shortest path) |

### A* vs Dijkstra Comparison

| Feature | A* | Dijkstra |
|---|---|---|
| Uses heuristic | ✅ Yes | ❌ No |
| Search direction | Guided toward goal | All directions equally |
| Nodes explored | Fewer | More |
| Optimal path | ✅ Yes | ✅ Yes |
| Speed | Faster | Slower |

---

## Results & Visualization

The algorithm is tested on a walled environment with:
- Map boundary: x ∈ [-10, 60], y ∈ [-10, 60]
- Multiple internal walls creating narrow corridors
- Start: (-5, -5) → Goal: (50, 50)

A* successfully navigates through all corridors and finds the globally optimal path.

---

## Future Development

This implementation provides a solid foundation that can be extended in several directions:

### 1. 🔢 Alternative Heuristics
Replace Euclidean distance with **Manhattan distance** for axis-aligned grids, or implement **weighted A*** (`f = g + w*h`, where `w > 1`) to trade optimality for speed in time-critical applications.

### 2. 🗺️ Continuous / Higher-Dimensional Space
Extend A* to work in **3D environments** for aerial robots (drones) or adapt it to **SE(2) / SE(3) configuration space** to account for robot orientation alongside position.

### 3. ⚡ Anytime A* (ARA\*)
Implement **Anytime Repairing A*** which first finds a sub-optimal path quickly, then progressively improves it — useful for real-time robotics where a fast initial plan is more important than perfection.

### 4. 🔄 Dynamic Replanning
Combine with **D\* Lite** principles to allow the robot to **replan on-the-fly** when new obstacles are detected by sensors during execution, without replanning from scratch.

### 5. 🤖 ROS2 Integration
Wrap this planner as a **ROS2 nav2 plugin** (`nav2_core::GlobalPlanner`) so it can be used as the global planner for a real mobile robot such as TurtleBot3, bridging simulation to real hardware.

### 6. 📊 Benchmark & Comparison Tool
Build an automated benchmark suite that compares A* against Dijkstra, RRT, and PRM on the same map — measuring metrics like path length, nodes expanded, and computation time.

### 7. 🧠 Learning-Based Heuristic
Use a **neural network** trained on solved instances to predict better heuristic values (Neural A*), drastically reducing the number of explored nodes in complex, high-dimensional environments.

---

## References

- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics.
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Motion Planning Project</i>
</div>
