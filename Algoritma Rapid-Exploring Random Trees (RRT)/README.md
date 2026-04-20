# Rapidly-Exploring Random Trees (RRT) Path Planning Algorithm

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![Library](https://img.shields.io/badge/Library-matplotlib%20%7C%20numpy%20%7C%20math%20%7C%20random-orange)
![Type](https://img.shields.io/badge/Type-Sampling--Based%20%7C%20Single--Query-red)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

*A single-query, tree-based sampling algorithm that rapidly explores the configuration space by growing a tree from the start toward random points, until the goal is reached.*

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
- [RRT vs PRM vs Grid-Based Planners](#rrt-vs-prm-vs-grid-based-planners)
- [Results & Visualization](#results--visualization)
- [Future Development](#future-development)

---

## Overview

**Rapidly-Exploring Random Trees (RRT)**, introduced by Steven LaValle in 1998, is one of the most influential algorithms in motion planning. Rather than constructing a dense roadmap like PRM, RRT builds a **single tree** from the start position that grows incrementally toward randomly sampled points in the configuration space.

RRT is a **single-query planner** — it is designed to solve one specific start-to-goal problem quickly, rather than building a reusable structure. Its key advantage is that it is **extremely fast** to produce a *feasible* (though not necessarily optimal) path, making it ideal for real-time robot navigation where a quick "good enough" path is preferred over a slow optimal one.

RRT is the foundation of a family of algorithms including RRT\*, RRT-Connect, Informed RRT\*, and MPPI — all widely used in production robotics systems today.

### Key Characteristics
| Property | Value |
|---|---|
| **Completeness** | Probabilistically complete |
| **Optimality** | Not guaranteed (sub-optimal paths) |
| **Planning Type** | Single-query |
| **Configuration Space** | 2D continuous (extensible to nD) |
| **Bias Strategy** | Goal sampling (`goal_sample_rate`%) |
| **Obstacles** | Circular (defined by center + radius) |

---

## How It Works

RRT grows a tree by repeatedly:
1. Sampling a random point in the space
2. Finding the nearest tree node
3. Steering toward the random point by a fixed step (`expand_dis`)
4. Checking for collisions
5. Adding the new node to the tree if collision-free

When a tree node gets close enough to the goal, a final connection attempt is made.

### Visual Representation

```
Start ●
       \
        ●──────● ← random sample pulled the tree this way
        |
        ●──────────●
                    \
              Obstacle ◎   ←  tree steers around it
                    /
                   ●──────● Goal ×
```

### Step-by-Step Process

```
1. Initialize tree with start node

2. For i = 1 to MAX_ITER:
   a. Sample random point q_rand:
      - With probability goal_sample_rate% → q_rand = goal
      - Otherwise → q_rand = random point in rand_area

   b. Find nearest node q_near in tree to q_rand

   c. Steer from q_near toward q_rand by expand_dis → q_new

   d. If q_new is:
      - Inside play area (if defined)
      - Collision-free with all obstacles
      → Add q_new to tree

   e. If distance(q_new, goal) ≤ expand_dis:
      → Try final connection to goal
      → If collision-free: PATH FOUND, reconstruct and return

3. Return None (no path found within MAX_ITER)
```

---

## Algorithm Details

### Steering Function
The `steer()` function moves from `from_node` toward `to_node` by exactly `expand_dis` (or the full distance if closer):

```python
def steer(self, from_node, to_node, extend_length=float("inf")):
    d, theta = self.calc_distance_and_angle(from_node, to_node)
    extend_length = min(extend_length, d)
    n_expand = math.floor(extend_length / self.path_resolution)
    
    for _ in range(n_expand):
        new_node.x += self.path_resolution * math.cos(theta)
        new_node.y += self.path_resolution * math.sin(theta)
```

The `path_resolution` controls the step size for intermediate collision checks — smaller values give more precise but slower collision detection.

### Goal Biasing
To avoid pure random exploration (which is very slow near the goal), the tree periodically **aims directly at the goal**:

```python
def get_random_node(self):
    if random.randint(0, 100) > self.goal_sample_rate:
        return random point in [min_rand, max_rand]
    else:
        return self.end  # bias toward goal
```

### Collision Checking (Circular Obstacles)
Each obstacle is defined as `(center_x, center_y, radius)`. Collision checking walks along each path segment:

```python
robot_radius = 0.5
for (ox, oy, size) in obstacleList:
    dx_list = [ox - x for x in node.path_x]
    dy_list = [oy - y for y in node.path_y]
    d_list = [dx**2 + dy**2 for dx, dy in zip(dx_list, dy_list)]
    if min(d_list) <= (size + robot_radius) ** 2:
        return False  # collision!
```

### Nearest Node Search
The nearest neighbor is found by linear scan — O(n) per query:

```python
@staticmethod
def get_nearest_node_index(node_list, rnd_node):
    dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
             for node in node_list]
    return dlist.index(min(dlist))
```

---

## Code Structure

```
rapid_exploring_random_trees.py
├── class RRT
│   ├── class Node
│   │   └── (x, y, path_x, path_y, parent)
│   │
│   ├── class AreaBounds
│   │   └── Defines play area boundaries (xmin, xmax, ymin, ymax)
│   │
│   ├── __init__(start, goal, obstacle_list, rand_area, ...)
│   │   └── Configures all planning parameters
│   │
│   ├── planning(animation=True)
│   │   └── Main RRT loop: sample → steer → check → add → repeat
│   │
│   ├── steer(from_node, to_node, extend_length)
│   │   └── Moves from_node toward to_node by extend_length
│   │
│   ├── generate_final_course(goal_ind)
│   │   └── Backtracks parent chain to reconstruct path
│   │
│   ├── calc_dist_to_goal(x, y)
│   │   └── Euclidean distance from (x,y) to goal
│   │
│   ├── get_random_node()
│   │   └── Random sample or goal (based on goal_sample_rate)
│   │
│   ├── draw_graph(rnd=None)
│   │   └── Animates tree growth and obstacles
│   │
│   ├── check_collision(node, obstacleList)
│   │   └── Checks all path segments against circular obstacles
│   │
│   ├── check_if_outside_play_area(node, play_area)
│   │   └── Validates node is within allowed region
│   │
│   └── calc_distance_and_angle(from_node, to_node)
│       └── Returns (distance, heading_angle)
│
└── main(gx=5.0, gy=8.0)
      └── Defines obstacles → creates RRT → plans → plots
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `start` | `[0, 0]` | Start position [x, y] |
| `goal` | `[5.0, 8.0]` | Goal position [x, y] |
| `rand_area` | `[-2, 15]` | Random sampling range [min, max] |
| `expand_dis` | `2.0` | Step size for tree expansion (meters) |
| `path_resolution` | `0.3` | Intermediate step size for collision check |
| `goal_sample_rate` | `10` | Probability (%) of sampling goal directly |
| `max_iter` | `1000` | Maximum number of iterations |
| `play_area` | `None` | Optional bounded region for tree growth |
| `show_animation` | `True` | Toggle real-time visualization |

---

## How to Run

### Prerequisites
```bash
pip install matplotlib numpy
```

### Execution
```bash
cd "Algoritma Rapid-Exploring Random Trees (RRT)"
python rapid_exploring_random_trees.py
```

### Expected Output
```
Start RRT planning
Found path!
```

A matplotlib window showing animated tree growth:
- **Green lines** → Tree branches growing from start
- **Blue circles** → Circular obstacles
- **Red X** → Start and goal positions
- **Red line** → Final path found

*Note: Due to random sampling, each run produces a different tree and path.*

---

## Complexity Analysis

| Metric | Value |
|---|---|
| **Time Complexity** | O(MAX_ITER × n) where n = current tree size |
| **Space Complexity** | O(MAX_ITER) — one node per iteration |
| **Path Quality** | Sub-optimal (not shortest path) |
| **Convergence** | Probabilistic — guaranteed to find path eventually |

### RRT vs PRM vs Grid-Based Planners

| Feature | RRT | PRM | A* / Dijkstra |
|---|---|---|---|
| Query type | Single | Multi (reusable roadmap) | Single |
| Path optimality | ❌ Sub-optimal | ❌ Approximate | ✅ Exact (on grid) |
| High-DOF support | ✅ Yes | ✅ Yes | ❌ Exponential |
| Speed to first solution | ✅ Very fast | Moderate | Moderate |
| Path smoothness | Jagged (needs smoothing) | Smoother | Grid-constrained |
| Dynamic obstacles | Limited | Limited | ✅ (D* Lite) |
| Memory usage | O(MAX_ITER) | O(N_SAMPLE) | O(V) |

---

## Results & Visualization

The test environment features:
- Clustered circular obstacles of varying sizes
- A large obstacle (radius 3) near the right boundary
- Start: (0, 0) → Goal: (5, 8)
- Map range: x, y ∈ [-2, 15]

RRT successfully navigates through the obstacle clusters by growing its tree around them. The path is non-optimal but found very quickly — typically within a few hundred iterations.

---

## Future Development

RRT is the foundation of a rich family of algorithms with many exciting extensions:

### 1. ⭐ RRT* (Asymptotically Optimal RRT)
Add a **rewiring step** after each node addition: check if existing nearby nodes can be reached at lower cost through the new node, and update parent pointers accordingly. **RRT\*** guarantees that the path cost converges to the true optimum as MAX_ITER → ∞.

### 2. 🎯 Informed RRT*
After the first feasible path is found, restrict future sampling to an **ellipsoidal subspace** that can potentially improve the solution. This focuses exploration where it matters, dramatically accelerating convergence to the optimum compared to plain RRT\*.

### 3. 🌲 RRT-Connect (Bidirectional RRT)
Grow **two trees simultaneously** — one from start and one from goal — and connect them when they get close enough. RRT-Connect converges to a solution significantly faster than single-tree RRT, making it ideal for high-dimensional manipulation planning.

### 4. 🧹 Path Shortcutting & Smoothing
Post-process the raw RRT path with a **path shortcutting** algorithm: repeatedly try to connect random pairs of path waypoints directly, skipping all intermediate nodes if the shortcut is collision-free. Follow with **B-spline or Bezier curve fitting** to produce smooth, robot-executable trajectories.

### 5. 🏎️ Kinodynamic RRT
Replace the simple steering function with a **differential equations-based kinodynamic model** — so the robot's velocity, acceleration, and turning radius constraints are respected during tree expansion. Essential for cars, drones, and any robot with non-holonomic constraints.

### 6. 🤖 ROS2 + MoveIt2 Integration
Register the RRT planner as a **MoveIt2 planning plugin** for a robotic arm. The configuration space becomes the arm's joint angles, and collision checking uses MoveIt2's collision detection. This enables planning for 6+ DOF manipulators on real hardware.

### 7. 🧠 Neural RRT (Learning-Guided Sampling)
Train a **neural network** on successful planning instances to learn which regions of space are most likely to contain useful samples. Replace the uniform sampler with this learned distribution to drastically reduce the iterations needed to reach the goal in complex environments.

### 8. 🌊 MPPI (Model Predictive Path Integral)
Use **GPU-parallel sampling** to simultaneously evaluate thousands of trajectory rollouts, selecting the lowest-cost bundle. MPPI is the next step beyond RRT for **agile, real-time** robot control — deployed on legged robots and racing drones.

---

## References

- LaValle, S. M. (1998). *Rapidly-Exploring Random Trees: A New Tool for Path Planning*. Technical Report, Iowa State University.
- LaValle, S. M., & Kuffner, J. J. (2001). *Randomized Kinodynamic Planning*. IJRR.
- Karaman, S., & Frazzoli, E. (2011). *Sampling-based algorithms for optimal motion planning*. IJRR.
- Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014). *Informed RRT\**: Optimal Incremental Path Planning Focused through an Admissible Ellipsoidal Heuristic. IROS 2014.
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Motion Planning Project</i>
</div>
