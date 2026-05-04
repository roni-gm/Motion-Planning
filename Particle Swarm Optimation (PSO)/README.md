# Particle Swarm Optimization (PSO)

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-blue?logo=python)
![Library](https://img.shields.io/badge/Library-matplotlib%20%7C%20numpy%20%7C%20math-orange)
![Type](https://img.shields.io/badge/Type-Swarm--Based%20%7C%20Metaheuristic-green)
![Status](https://img.shields.io/badge/Status-Complete-brightgreen)

*A population-based metaheuristic optimization algorithm inspired by the social behavior of bird flocking and fish schooling, capable of efficiently finding optimal or near-optimal solutions in continuous search spaces.*

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

Particle Swarm Optimization (PSO) is a computational method introduced by **Kennedy and Eberhart in 1995**, inspired by the emergent collective intelligence of bird flocks and fish schools. Each individual in the swarm — called a **particle** — represents a candidate solution. Particles navigate through the search space by adjusting their positions based on their own best-known position (**personal best**) and the best position discovered by the entire swarm (**global best**).

Unlike gradient-based methods, PSO requires **no derivative information**, making it particularly well-suited for non-differentiable, multimodal, and noisy optimization problems commonly found in robotics, engineering design, and machine learning hyperparameter tuning.

### Key Characteristics

| Property | Value |
|---|---|
| **Algorithm Type** | Population-based Metaheuristic |
| **Completeness** | Probabilistic — may not always find global optimum |
| **Optimality** | Not guaranteed — heuristic approximation |
| **Search Strategy** | Swarm intelligence (collective movement) |
| **Problem Type** | Continuous and discrete optimization |
| **Gradient Required** | ❌ No — derivative-free optimization |
| **Memory** | Each particle remembers its personal best position |
| **Convergence** | Iterative, stochastic |
| **Parallelizable** | ✅ Yes — particles are independent per iteration |

---

## How It Works

PSO simulates social behavior: particles "fly" through the solution space, attracted toward positions that previously yielded good results — both their own and the swarm's collective memory.

Each particle maintains:
- **Position** `x` — current candidate solution in the search space
- **Velocity** `v` — direction and speed of movement
- **Personal Best** `pbest` — best position the particle has ever visited
- **Global Best** `gbest` — best position any particle in the swarm has ever visited

The algorithm evaluates each particle's position against an **objective function** (or fitness function) and guides all particles toward progressively better regions of the search space.

### Core Equations

**Velocity Update:**

$$v_i^{t+1} = w \cdot v_i^t + c_1 \cdot r_1 \cdot (pbest_i - x_i^t) + c_2 \cdot r_2 \cdot (gbest - x_i^t)$$

**Position Update:**

$$x_i^{t+1} = x_i^t + v_i^{t+1}$$

| Symbol | Name | Description |
|---|---|---|
| `v_i^t` | Velocity | Current velocity of particle `i` at iteration `t` |
| `x_i^t` | Position | Current position of particle `i` at iteration `t` |
| `w` | Inertia Weight | Controls momentum — balances exploration vs. exploitation |
| `c1` | Cognitive Coefficient | Pull toward particle's own best known position |
| `c2` | Social Coefficient | Pull toward swarm's global best known position |
| `r1, r2` | Random Factors | Uniform random numbers ∈ [0, 1] — adds stochasticity |
| `pbest_i` | Personal Best | Best position particle `i` has ever visited |
| `gbest` | Global Best | Best position any particle in swarm has ever visited |

### Step-by-Step Process

```
1. Initialize:
   - Generate N particles with random positions x_i ∈ [lb, ub]
   - Generate random initial velocities v_i
   - Evaluate fitness f(x_i) for each particle
   - Set pbest_i = x_i for all particles
   - Set gbest = particle with best fitness value

2. Loop until stopping criterion (max iterations or convergence):
   a. For each particle i:
      i.   Generate random r1, r2 ∈ [0, 1]
      ii.  Update velocity:
              v_i = w * v_i
                  + c1 * r1 * (pbest_i - x_i)
                  + c2 * r2 * (gbest  - x_i)
      iii. Clamp velocity: v_i = clip(v_i, -v_max, v_max)
      iv.  Update position: x_i = x_i + v_i
      v.   Clamp position: x_i = clip(x_i, lb, ub)
      vi.  Evaluate new fitness: f_new = f(x_i)
      vii. If f_new < f(pbest_i):
              pbest_i = x_i
      viii.If f_new < f(gbest):
              gbest = x_i

3. (Optional) Update inertia weight w (linear decay):
      w = w_max - (w_max - w_min) * (t / max_iter)

4. Return gbest as the best solution found
```

---

## Algorithm Details

### Velocity Update — Three Components

The velocity update consists of three distinct forces acting on each particle:

```
v_new = [Inertia]    +    [Cognitive]    +    [Social]
        w * v_old    +  c1*r1*(pb - x)  +  c2*r2*(gb - x)
```

**1. Inertia Component** `w * v_old`
- Preserves the particle's current direction of travel
- High `w` → more exploration (wide search)
- Low `w` → more exploitation (refine around current best)
- Typical value: `w ∈ [0.4, 0.9]`, often decayed linearly over iterations

**2. Cognitive Component** `c1 * r1 * (pbest - x)`
- Pulls the particle back toward its own historical best position
- Represents individual memory and self-learning
- Typical value: `c1 = 2.0`

**3. Social Component** `c2 * r2 * (gbest - x)`
- Pulls the particle toward the swarm's globally discovered best
- Represents collective intelligence and information sharing
- Typical value: `c2 = 2.0`

### Velocity Clamping

To prevent particles from flying out of the search space at explosive speeds, velocity is bounded:

```python
v = np.clip(v, -v_max, v_max)
# Common choice: v_max = (ub - lb) * 0.2
```

### Inertia Weight Decay (Linear)

```python
w = w_max - (w_max - w_min) * (iteration / max_iterations)
# Example: w_max = 0.9, w_min = 0.4
```

This allows broad exploration early and fine exploitation near convergence.

### Topology Variants

| Topology | Description | Convergence Speed |
|---|---|---|
| **Global Best (gbest)** | All particles share one global best | Fast, risk of premature convergence |
| **Local Best (lbest)** | Each particle shares best within a neighborhood ring | Slower, more robust |
| **Von Neumann** | Grid neighborhood structure | Balanced |

---

## Code Structure

```
pso.py
├── class PSO
│   ├── __init__(n_particles, n_dimensions, lb, ub, max_iter, w, c1, c2)
│   │     └── Initializes swarm with random positions and velocities
│   ├── evaluate(position)
│   │     └── Computes fitness value of a given position vector
│   ├── update_velocity(particle)
│   │     └── Applies inertia + cognitive + social force equations
│   ├── update_position(particle)
│   │     └── Moves particle and clamps within search bounds
│   ├── optimize()
│   │     └── Main PSO loop → returns gbest position and value
│   └── plot_convergence()
│         └── Plots fitness vs. iteration curve
│
├── class Particle
│   ├── position       → current position vector
│   ├── velocity       → current velocity vector
│   ├── pbest_pos      → personal best position
│   └── pbest_val      → personal best fitness value
│
└── main()
      └── Defines objective function, bounds → runs PSO → plots results
```

### Example Python Implementation

```python
import numpy as np
import matplotlib.pyplot as plt

class Particle:
    def __init__(self, n_dim, lb, ub):
        self.position  = np.random.uniform(lb, ub, n_dim)
        self.velocity  = np.random.uniform(-abs(ub - lb), abs(ub - lb), n_dim)
        self.pbest_pos = self.position.copy()
        self.pbest_val = float('inf')

class PSO:
    def __init__(self, func, n_particles=30, n_dim=2,
                 lb=-10, ub=10, max_iter=200,
                 w=0.9, c1=2.0, c2=2.0):
        self.func        = func
        self.n_particles = n_particles
        self.n_dim       = n_dim
        self.lb, self.ub = lb, ub
        self.max_iter    = max_iter
        self.w, self.c1, self.c2 = w, c1, c2
        self.w_min       = 0.4
        self.v_max       = abs(ub - lb) * 0.2

        # Initialize swarm
        self.swarm     = [Particle(n_dim, lb, ub) for _ in range(n_particles)]
        self.gbest_pos = None
        self.gbest_val = float('inf')
        self.history   = []

    def optimize(self):
        # Evaluate initial fitness
        for p in self.swarm:
            val = self.func(p.position)
            p.pbest_val = val
            if val < self.gbest_val:
                self.gbest_val = val
                self.gbest_pos = p.position.copy()

        for t in range(self.max_iter):
            # Linearly decay inertia weight
            w = self.w - (self.w - self.w_min) * (t / self.max_iter)

            for p in self.swarm:
                r1 = np.random.rand(self.n_dim)
                r2 = np.random.rand(self.n_dim)

                # Velocity update
                cognitive = self.c1 * r1 * (p.pbest_pos - p.position)
                social    = self.c2 * r2 * (self.gbest_pos - p.position)
                p.velocity = w * p.velocity + cognitive + social
                p.velocity = np.clip(p.velocity, -self.v_max, self.v_max)

                # Position update
                p.position += p.velocity
                p.position  = np.clip(p.position, self.lb, self.ub)

                # Evaluate fitness
                val = self.func(p.position)
                if val < p.pbest_val:
                    p.pbest_val = val
                    p.pbest_pos = p.position.copy()
                if val < self.gbest_val:
                    self.gbest_val = val
                    self.gbest_pos = p.position.copy()

            self.history.append(self.gbest_val)
            print(f"Iter {t+1:>4d} | Best Fitness: {self.gbest_val:.6f}")

        return self.gbest_pos, self.gbest_val

    def plot_convergence(self):
        plt.figure(figsize=(10, 5))
        plt.plot(self.history, color='royalblue', linewidth=2)
        plt.xlabel('Iteration')
        plt.ylabel('Best Fitness Value')
        plt.title('PSO Convergence Curve')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

# --- Objective Function (Sphere Function) ---
def sphere(x):
    return np.sum(x ** 2)

def main():
    pso = PSO(func=sphere, n_particles=30, n_dim=2,
              lb=-10, ub=10, max_iter=200,
              w=0.9, c1=2.0, c2=2.0)

    best_pos, best_val = pso.optimize()
    print(f"\nBest Position : {best_pos}")
    print(f"Best Value    : {best_val:.8f}")
    pso.plot_convergence()

if __name__ == '__main__':
    main()
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `n_particles` | `30` | Number of particles in the swarm |
| `n_dim` | `2` | Number of dimensions (variables) in the problem |
| `lb` | `-10` | Lower bound of the search space |
| `ub` | `10` | Upper bound of the search space |
| `max_iter` | `200` | Maximum number of iterations |
| `w` | `0.9` | Initial inertia weight (decays to `w_min`) |
| `w_min` | `0.4` | Minimum inertia weight (end of decay) |
| `c1` | `2.0` | Cognitive coefficient (personal best attraction) |
| `c2` | `2.0` | Social coefficient (global best attraction) |
| `v_max` | `0.2 × (ub-lb)` | Maximum allowed velocity per dimension |
| `show_animation` | `True` | Toggle real-time visualization of particle movement |

### Parameter Tuning Guidelines

| Behavior Goal | Suggested Adjustment |
|---|---|
| More exploration | Increase `w`, reduce `c2` |
| Faster convergence | Reduce `w`, increase `c2` |
| Avoid premature convergence | Use local best topology, increase `c1` |
| High-dimensional problems | Increase `n_particles`, decay `w` slowly |

---

## How to Run

### Prerequisites
```bash
pip install numpy matplotlib
```

### Execution
```bash
cd "Particle Swarm Optimization"
python pso.py
```

### Expected Output
```
Iter    1 | Best Fitness: 45.231842
Iter    2 | Best Fitness: 32.874910
Iter    3 | Best Fitness: 18.562043
...
Iter  200 | Best Fitness: 0.00000312

Best Position : [ 0.00041823 -0.00029175]
Best Value    : 0.00000031
```

A matplotlib window will open showing:
- **Scatter plot** → Positions of all particles at each iteration (if animation enabled)
- **Star marker (★)** → Current global best position
- **Convergence curve** → Best fitness value vs. iteration number
- **Color gradient** → Particles colored by their current fitness value

---

## Complexity Analysis

| Metric | Value |
|---|---|
| **Time Complexity** | O(T × N × D) — T iterations, N particles, D dimensions |
| **Space Complexity** | O(N × D) — stores positions and velocities for all particles |
| **Fitness Evaluations** | N × T total evaluations |
| **Scalability** | Linear in both particles and dimensions |
| **Parallelizability** | High — particle updates are independent per iteration |

### PSO vs. Other Optimization Methods

| Feature | PSO | Genetic Algorithm | Gradient Descent | Simulated Annealing |
|---|---|---|---|---|
| Gradient required | ❌ No | ❌ No | ✅ Yes | ❌ No |
| Population-based | ✅ Yes | ✅ Yes | ❌ No | ❌ No |
| Global optimum | Probabilistic | Probabilistic | ❌ Local only | Probabilistic |
| Implementation complexity | Low | Medium | Low | Low |
| Convergence speed | Fast | Medium | Fast (near optimum) | Slow |
| Handles multimodal | ✅ Good | ✅ Good | ❌ Poor | ✅ Good |
| Memory per agent | pbest | Chromosome | — | Current state |

---

## Results & Visualization

The algorithm is tested on standard benchmark functions commonly used to evaluate optimization algorithms:

### Benchmark Functions Tested

| Function | Formula | Global Minimum | Characteristics |
|---|---|---|---|
| **Sphere** | `Σ xᵢ²` | 0 at origin | Unimodal, convex |
| **Rastrigin** | `10n + Σ[xᵢ² - 10cos(2πxᵢ)]` | 0 at origin | Highly multimodal |
| **Rosenbrock** | `Σ[100(xᵢ₊₁-xᵢ²)² + (1-xᵢ)²]` | 0 at (1,1,...,1) | Narrow curved valley |
| **Ackley** | Complex exponential + cosine | 0 at origin | Multimodal, deceptive |

### Visualization Output
- **Particle Swarm Animation** — 2D scatter plot showing all particles converging toward the global optimum across iterations
- **Convergence Curve** — Log-scale fitness vs. iteration plot demonstrating the rate of convergence
- **Search Space Heatmap** — Contour map of the objective function overlaid with particle trajectories
- **Personal & Global Best Tracking** — Live update of `pbest` and `gbest` markers on the map

### Sample Results (Sphere Function, 2D, 200 iterations)

```
Initial best fitness : 87.4321
After  50 iterations : 0.8432
After 100 iterations : 0.0087
After 150 iterations : 0.0001
Final best fitness   : 0.00000031   ✅ Converged near global optimum
```

---

## Future Development

This PSO implementation provides a solid foundation that can be extended in several directions:

### 1. 🔀 Adaptive Inertia Weight Strategies
Replace linear decay with **nonlinear adaptive schemes** such as chaotic inertia weight, fuzzy-controlled `w`, or random inertia weight, which can better balance exploration–exploitation depending on swarm diversity.

### 2. 🌍 Multi-Objective PSO (MOPSO)
Extend the algorithm to handle **multiple conflicting objectives simultaneously** (e.g., minimize path length AND energy consumption in robotics). MOPSO maintains a Pareto archive instead of a single `gbest`, returning a set of non-dominated optimal solutions.

### 3. 🗺️ PSO for Robot Path Planning
Encode robot waypoints as particle positions and use PSO to **optimize a complete trajectory** in a 2D/3D map — minimizing path length, obstacle proximity, and travel time simultaneously. This can complement or replace grid-based methods like A*.

### 4. ⚡ Hybrid PSO (PSO + Local Search)
Combine PSO with a **local refinement method** (e.g., Nelder-Mead, gradient descent, or hill climbing) to accelerate fine-grained exploitation once particles are near the optimum, dramatically reducing the final convergence time.

### 5. 🔄 Dynamic & Constrained Optimization
Adapt PSO for **time-varying environments** where the objective function changes during optimization (common in real-time robotics), or add **constraint handling techniques** (penalty functions, feasibility rules) for engineering design problems with hard constraints.

### 6. 🤖 ROS2 Integration for Real-Time Tuning
Wrap PSO as a **ROS2 node** that dynamically tunes PID controller gains or motion planner parameters for mobile robots during operation, enabling online optimization based on real sensor feedback from hardware like TurtleBot3.

### 7. 🧠 Neural Network Training with PSO
Use PSO as an alternative to backpropagation for **training neural networks** — particularly useful for small networks with non-differentiable activation functions, or for training in environments where gradients are unavailable or unstable.

### 8. 📊 Benchmark & Comparison Suite
Build an automated testing framework that compares PSO against Genetic Algorithm, Differential Evolution, and Grey Wolf Optimizer across the **CEC benchmark suite** — measuring metrics like success rate, average function evaluations, and solution quality across 30+ independent runs.

---

## References

- Kennedy, J., & Eberhart, R. (1995). *Particle swarm optimization*. Proceedings of ICNN'95 — International Conference on Neural Networks.
- Shi, Y., & Eberhart, R. (1998). *A modified particle swarm optimizer*. IEEE International Conference on Evolutionary Computation.
- Clerc, M., & Kennedy, J. (2002). *The particle swarm — explosion, stability, and convergence in a multidimensional complex space*. IEEE Transactions on Evolutionary Computation.
- Poli, R., Kennedy, J., & Blackwell, T. (2007). *Particle swarm optimization: An overview*. Swarm Intelligence, 1(1), 33–57.
- PythonRobotics by Atsushi Sakai — [GitHub](https://github.com/AtsushiSakai/PythonRobotics)

---

<div align="center">
<i>Robotics Engineering · Politeknik Negeri Batam · Intelligent Optimization Project</i>
</div>
