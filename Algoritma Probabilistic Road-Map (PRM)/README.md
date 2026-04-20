# Probabilistic Road Map (PRM) Path Planning Algorithm

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![Library](https://img.shields.io/badge/Library-numpy%20%7C%20scipy%20%7C%20matplotlib-orange)
![Type](https://img.shields.io/badge/Type-Sampling--Based%20%7C%20Multi--Query-purple)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

*A sampling-based motion planner that constructs a probabilistic roadmap through random sampling, enabling efficient path planning in high-dimensional and complex configuration spaces.*

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
- [PRM vs Grid-Based Planners](#prm-vs-grid-based-planners)
- [Results & Visualization](#results--visualization)
- [Future Development](#future-development)

---

## Overview

The **Probabilistic Road Map (PRM)** algorithm, introduced by Kavraki et al. in 1996, belongs to the family of **sampling-based motion planners**. Unlike grid-based methods (Dijkstra, A*, D*) that discretize the entire environment into a fixed grid, PRM works by **randomly sampling** points in the free configuration space and connecting nearby samples into a reachable roadmap graph.

This approach makes PRM particularly powerful for:
- **High-dimensional** configuration spaces (robotic arms with many DOF)
- **Large-scale** environments where a full grid would be too memory-intensive
- **Multi-query** scenarios where the same roadmap is reused for many start/goal pairs

This implementation uses **A\* search on the constructed roadmap** to find the shortest path, combining the scalability of sampling-based exploration with the informed search efficiency of A\*.

> **Two variants are implemented in this folder:**
> - `probabilistic_road_map_aStar.py` — PRM with A* graph search *(this document)*
> - `probabilistic_road_map_dijkstra.py` — PRM with Dijkstra graph search

### Key Characteristics
| Property | Value |
|---|---|
| **Completeness** | Probabilistically complete |
| **Optimality** | Not guaranteed (depends on sample density) |
| **Planning Type** | Multi-query (roadmap reusable) |
| **Configuration Space** | 2D continuous (extensible to nD) |
| **Graph Search** | A* (Euclidean heuristic) |
| **Collision Checking** | Step-wise ray casting + KD-Tree |

---

## How It Works

PRM consists of two distinct phases:

### Phase 1: Learning Phase — Building the Roadmap

```
1. Randomly sample N_SAMPLE collision-free points in the environment
2. For each sampled point:
   a. Find its K nearest neighbors (using KD-Tree for efficiency)
   b. For each neighbor, check if the straight-line edge is collision-free
   c. If collision-free and within MAX_EDGE_LEN → add edge to roadmap
3. Add the start and goal nodes to the roadmap
```

### Phase 2: Query Phase — Finding the Path

```
4. Run A* search on the constructed roadmap graph
5. Return the sequence of sampled waypoints forming the path
```

### Visual Representation
```
Obstacle    Free Space    Sampled Node    Edge
   ▓           ·              ●          ────

  ▓▓▓▓▓    ·  ●──────●  ·   ▓▓▓
  ▓▓▓▓▓   ·  /   ·    \  ·  ▓▓▓
  ▓▓▓▓▓  · ●    ·    ●──────●  ← Start node added
         ·  \  ·   /        ·
         ·   ●─────●        ·
         ·        ↑ Goal node added
```

---

## Algorithm Details

### Random Sampling
Nodes are sampled uniformly at random within the map bounds, and rejected if they fall inside an obstacle (checked via KD-Tree distance to obstacle points):

```python
while len(sample_x) <= N_SAMPLE:
    tx = (rng.random() * (max_x - min_x)) + min_x
    ty = (rng.random() * (max_y - min_y)) + min_y
    dist, _ = obstacle_kd_tree.query([tx, ty])
    if dist >= robot_radius:   # collision-free
        sample_x.append(tx)
        sample_y.append(ty)
```

### KD-Tree for Efficient Neighbor Search
Instead of brute-force O(n²) neighbor queries, this implementation uses **`scipy.spatial.KDTree`** which reduces nearest-neighbor lookup to O(log n):

```python
obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)
sample_kd_tree   = KDTree(np.vstack((sample_x, sample_y)).T)

dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
```

### Collision Checking (Step-wise Ray Casting)
Edges between nodes are checked by marching along the line segment at step size `robot_radius` and querying the obstacle KD-Tree at each step:

```python
def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    # Step along edge, check each point against obstacles
    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True   # collision detected
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)
```

### A* Search on Roadmap
The graph search on the roadmap uses A* with Euclidean heuristic, selecting nodes by `f = cost + h(node, goal)`:

```python
c_id = min(open_set, key=lambda o: open_set[o].cost + open_set[o].h)
```

---

## Code Structure

```
probabilistic_road_map_aStar.py
├── Constants
│   ├── N_SAMPLE = 1000       ← Number of random samples
│   ├── N_KNN = 10            ← Max edges per node
│   └── MAX_EDGE_LEN = 30.0   ← Maximum edge length [m]
│
├── class Node
│   └── (x, y, cost, h, parent_index)
│
├── heuristic(x, y, gx, gy)
│   └── Euclidean distance to goal
│
├── prm_planning(start, goal, obstacles, robot_radius)
│   ├── Builds obstacle KD-Tree
│   ├── sample_points()        ← Random free-space sampling
│   ├── generate_road_map()    ← KNN edge generation
│   └── astar_planning()       ← A* search on roadmap
│
├── is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree)
│   └── Step-wise edge collision checker
│
├── generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree)
│   └── Builds adjacency list (roadmap graph)
│
├── astar_planning(sx, sy, gx, gy, road_map, sample_x, sample_y)
│   └── A* search returning (rx, ry) path
│
├── sample_points(sx, sy, gx, gy, rr, ox, oy, kd_tree, rng)
│   └── Uniform random sampling with collision rejection
│
└── main(rng=None)
      └── Defines map → runs PRM → plots roadmap and path
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `N_SAMPLE` | `1000` | Number of random configuration samples |
| `N_KNN` | `10` | Maximum number of neighbors per node |
| `MAX_EDGE_LEN` | `30.0` | Maximum allowed edge length (meters) |
| `sx, sy` | `-5.0, -5.0` | Start position (meters) |
| `gx, gy` | `50.0, 50.0` | Goal position (meters) |
| `robot_size` | `1.0` | Robot radius for collision checking |
| `show_animation` | `True` | Toggle real-time visualization |

---

## How to Run

### Prerequisites
```bash
pip install matplotlib numpy scipy
```

### Execution — PRM with A* Search
```bash
cd "Algoritma Probabilistic Road-Map (PRM)"
python probabilistic_road_map_aStar.py
```

### Execution — PRM with Dijkstra Search
```bash
python probabilistic_road_map_dijkstra.py
```

### Expected Output
A matplotlib window showing:
- **Black dots** → Obstacles / walls
- **Blue dots** → Randomly sampled free-space nodes
- **Black lines** → Roadmap edges (collision-free connections)
- **Green circle** → Start position
- **Blue X** → Goal position
- **Green X marks** → Nodes explored during A* search on roadmap
- **Red line** → Final path through roadmap

*Note: Because sampling is random, each run may produce a slightly different roadmap and path.*

---

## Complexity Analysis

| Phase | Complexity |
|---|---|
| **Sampling** | O(N_SAMPLE) |
| **KD-Tree construction** | O(N log N) |
| **Roadmap construction** | O(N · K · L) where L = edge length / step size |
| **A* on roadmap** | O((N + E) log N) |
| **Total** | O(N · K · L + N log N) |

### PRM vs Grid-Based Planners

| Feature | PRM | A* / Dijkstra |
|---|---|---|
| Space representation | Continuous (samples) | Discrete (grid) |
| Memory usage | O(N_SAMPLE) | O(grid_width × grid_height) |
| Path optimality | Approximate | Exact (on grid) |
| Handles high-DOF | ✅ Yes | ❌ Exponentially expensive |
| Multi-query reuse | ✅ Yes (roadmap is prebuilt) | ❌ No (must replan each time) |
| Completeness | Probabilistic | Complete |
| Path smoothness | Smoother (continuous) | Staircase (grid-constrained) |

---

## Results & Visualization

The test environment features:
- Map boundary: x ∈ [-10, 60], y ∈ [-10, 60]
- Multiple internal walls with narrow passages
- Start: (-5, -5) → Goal: (50, 50)
- 1000 random samples with up to 10 neighbors each

PRM successfully builds a roadmap that spans the entire free space, and A* finds a path through the roadmap waypoints. The path quality improves with more samples (`N_SAMPLE`).

---

## Future Development

PRM's sampling-based nature opens many exciting directions:

### 1. 🤖 High-Dimensional Arm Configuration Space
Extend PRM from 2D (x, y) to **robot arm joint space** (e.g., 6-DOF). Each sample becomes a joint angle vector instead of a 2D point, and collision checking uses **forward kinematics** to check if the robot body intersects any obstacle — the exact use case PRM was designed for.

### 2. 🏎️ Lazy PRM
Defer collision checking of edges until query time — build the roadmap first with unchecked edges, then only validate the edges along the planned path. **Lazy PRM** significantly speeds up the learning phase when collision checking is expensive (e.g., 3D mesh environments).

### 3. ⭐ PRM* (Asymptotically Optimal PRM)
Replace fixed-K nearest neighbor with a **radius-based connection** where the radius shrinks as samples increase. **PRM\*** is proven to be **asymptotically optimal** — the path quality converges to the true optimum as N_SAMPLE → ∞.

### 4. 🌿 Visibility-Based PRM (VPRM)
Construct a **minimal roadmap** by only placing nodes at key visibility points (e.g., corners of obstacles). This dramatically reduces the number of nodes required to achieve full coverage, improving query speed on structured environments.

### 5. 🔄 Dynamic PRM with Node Removal
Extend the roadmap to support **real-time obstacle updates**: when a new obstacle is detected, remove the roadmap nodes and edges inside it and rerun A* on the remaining graph. This bridges PRM and dynamic planners like D\* Lite.

### 6. 🤖 Integration with Robotic Arm Simulation (MoveIt2)
Wrap the PRM planner as a **MoveIt2 planning plugin** for robotic arm motion planning. Replace the 2D grid with a MoveIt2 collision checker and run PRM directly in joint configuration space for a 6-DOF manipulator.

### 7. 📊 Adaptive Sampling Density
Implement **bridge sampling** or **Gaussian sampling** to place more samples near obstacle boundaries — the narrow passage problem — and fewer in wide open areas. This drastically improves the planner's ability to find paths through tight corridors.

---

## References

- Kavraki, L. E., Svestka, P., Latombe, J. C., & Overmars, M. H. (1996). *Probabilistic roadmaps for path planning in high-dimensional configuration spaces*. IEEE Transactions on Robotics and Automation.
- Karaman, S., & Frazzoli, E. (2011). *Sampling-based algorithms for optimal motion planning*. IJRR.
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Motion Planning Project</i>
</div>
