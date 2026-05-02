# 🗺️ Trajectory Planner — A* vs RRT

![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)
![NumPy](https://img.shields.io/badge/NumPy-2.4-013243?logo=numpy)
![Matplotlib](https://img.shields.io/badge/Matplotlib-3.10-11557c)

An interactive 2D path-planning application that compares **A\*** (grid-based) and **RRT** (sampling-based) algorithms side by side, featuring:

- **A\* Search** with three configurable heuristics (Manhattan, Euclidean, Diagonal)
- **RRT** with tunable step size, goal bias, and collision detection
- **Interactive environment editor** — draw obstacles, set start/goal with the mouse
- **Dynamic obstacles** — linear (bouncing) and circular (orbital)
- **Step-by-step animation** of both algorithms running simultaneously
- **Quantitative comparison** — path length, computation time, nodes explored
- **62 unit tests** with 100 % pass rate

> Built as a portfolio project demonstrating path-planning algorithms from first principles using only NumPy and Matplotlib.

---

## 📐 Mathematical Foundation

### 1. A\* Search Algorithm

A\* finds the shortest path on a weighted graph by expanding the node with the lowest evaluation function:

```
f(n) = g(n) + h(n)
```

| Symbol | Meaning |
|--------|---------|
| **g(n)** | Actual cost from start to node n |
| **h(n)** | Heuristic estimate from n to goal |
| **f(n)** | Total estimated cost through n |

The algorithm maintains an **open set** (priority queue) and a **closed set** (already expanded). At each step it expands the node with minimum f, updates neighbours, and stops when the goal is reached.

**8-connected grid movement** — each cell connects to 8 neighbours with costs:

```
Cardinal (↑↓←→):  cost = 1
Diagonal (↗↘↙↖):  cost = √2 ≈ 1.414
```

#### Heuristic Functions

All heuristics estimate the remaining distance from node `(r₁, c₁)` to goal `(r₂, c₂)`:

| Heuristic | Formula | Admissible (8-conn)? |
|-----------|---------|---------------------|
| **Manhattan** | `\|Δr\| + \|Δc\|` | ❌ Overestimates diagonals |
| **Euclidean** | `√(Δr² + Δc²)` | ✅ Always ≤ true cost |
| **Diagonal** | `(Δr + Δc) + (√2 − 2) · min(Δr, Δc)` | ✅ Exact for 8-conn |

A heuristic is **admissible** when `h(n) ≤ h*(n)` (never overestimates the true optimal cost). With an admissible heuristic, A\* guarantees an optimal path.

---

### 2. RRT — Rapidly-exploring Random Tree

RRT builds a tree by iteratively sampling random points and extending the nearest node toward them:

```
for i = 1 to max_iterations:
    x_rand ← SAMPLE(space, goal_bias)
    x_near ← NEAREST(tree, x_rand)
    x_new  ← STEER(x_near, x_rand, step_size)
    if COLLISION_FREE(x_near, x_new):
        tree.add(x_new, parent=x_near)
        if ‖x_new − x_goal‖ ≤ threshold:
            return TRACE_PATH(tree, x_new)
```

| Parameter | Effect |
|-----------|--------|
| **step_size** | Maximum extension distance per iteration |
| **goal_bias** | Probability [0–1] of sampling x_goal instead of random point |
| **goal_threshold** | Distance at which the goal is considered reached |
| **max_iter** | Upper limit on tree expansion iterations |

**Collision detection** uses the Liang–Barsky line-clipping algorithm to test segment–rectangle intersection in O(1) per obstacle.

#### Key properties

| Property | A\* | RRT |
|----------|-----|-----|
| Completeness | ✅ (finite grid) | Probabilistically complete |
| Optimality | ✅ (admissible h) | ❌ (suboptimal; RRT\* needed) |
| Space | Discrete grid | Continuous |
| Best for | Small/medium grids | High-dimensional / continuous spaces |

---

### 3. Dynamic Obstacles

Two motion models are implemented:

**Linear (bouncing):** obstacle translates at constant velocity `(dx, dy)` and reverses direction on world boundary collision.

**Circular (orbital):** obstacle follows a circular orbit of radius `r` around centre `(cx, cy)`:

```
x(t) = cx + r·cos(θ₀ + ω·t)
y(t) = cy + r·sin(θ₀ + ω·t)
```

where `ω` is the angular speed (radians/tick).

---

## 🗂️ Project Structure

```
trajectory_planner/
├── main.py                   # Application entry point
├── config.py                 # Global constants and default parameters
├── requirements.txt          # Python dependencies
├── environment/
│   ├── __init__.py
│   ├── base_env.py           # Abstract base environment
│   ├── grid_env.py           # Grid-based environment with collision detection
│   ├── obstacle.py           # Static and dynamic obstacle classes
│   └── map_presets.py        # Predefined environment configurations
├── algorithms/
│   ├── __init__.py
│   ├── base_planner.py       # Abstract base planner interface
│   ├── astar.py              # A* search with multiple heuristics
│   └── rrt.py                # Rapidly-exploring Random Tree
├── metrics/
│   ├── __init__.py
│   └── evaluator.py          # Metrics collection and comparison
├── tests/
│   └── test_planner.py       # 62 unit tests (pytest)
└── ui/
    ├── __init__.py
    ├── app.py                # Main application orchestrator
    ├── canvas_widget.py      # Matplotlib canvas with mouse interaction
    ├── control_panel.py      # Parameter controls and action buttons
    └── metrics_panel.py      # Comparison charts and tables
```

---

## 🚀 Quick Start

```bash
# 1. Clone and install
git clone https://github.com/jams286/trajectory_planner.git
cd trajectory_planner
python -m venv venv

# Windows
venv\Scripts\activate
# macOS / Linux
source venv/bin/activate

pip install -r requirements.txt

# 2. Run the application
python main.py

# 3. Run tests
pip install pytest
pytest tests/ -v
```

---

## 🎮 Usage

### Controls

| Action | How |
|---|---|
| **Draw obstacle** | Select "Draw" mode → click and drag on canvas |
| **Erase obstacle** | Select "Erase" mode → click on canvas |
| **Set start point** | Select "Set Start" mode → click on canvas |
| **Set goal point** | Select "Set Goal" mode → click on canvas |
| **Run both algorithms** | Click "▶ Run Both" button |
| **Step through** | Click "⏭ Step" for one iteration at a time |
| **Pause / Resume** | Click "⏸ Pause" during animation |
| **Reset** | Click "↺ Reset" to clear paths and restore environment |
| **Change speed** | Adjust the Steps/Frame and Interval sliders |
| **Load preset** | Select from the Map Preset dropdown |

### Algorithm Parameters

**A\***
- Heuristic: Manhattan · Euclidean · Diagonal

**RRT**
- Step size: distance extended per iteration (0.5–5.0)
- Max iterations: upper limit on tree expansion (500–20 000)
- Goal bias: probability [0–1] of sampling toward the goal
- Goal threshold: distance to consider goal reached (0.5–5.0)

---

## 📊 Test Results

```
62 passed

TestStaticObstacle          (6 tests)  — containment, rasterization, properties
TestDynamicLinearObstacle   (3 tests)  — movement, boundary bounce, reset
TestDynamicCircularObstacle (3 tests)  — initial position, orbit update, reset
TestSegmentRectIntersect    (4 tests)  — through, miss, inside, horizontal
TestGridEnvironment        (10 tests)  — free cells, bounds, neighbours, obstacles, segments, dynamic update
TestMapPresets              (8 tests)  — all presets return valid obstacle lists and environments
TestAStarPlanner            (8 tests)  — path finding, start/goal correctness, blocked, all heuristics, step generator
TestHeuristics              (5 tests)  — values, admissibility, overestimation
TestRRTPlanner              (5 tests)  — path finding, start position, tree growth, goal bias effect
TestMetrics                 (6 tests)  — path length, collector, comparison dict, reset, timer tracking
TestPlannerState            (2 tests)  — default and custom state
```

---

## 📊 Metrics Comparison

The application computes and displays the following metrics for each algorithm:

| Metric | Description |
|---|---|
| **Path Length** | Total Euclidean distance of the solution path |
| **Compute Time** | Wall-clock time to find the solution (seconds) |
| **Nodes Explored** | Number of nodes expanded (A\*) or added to tree (RRT) |

Example comparison on the Maze preset (50×50 grid):

| Metric | A\* (Euclidean) | RRT |
|---|---|---|
| Path Length | 70.08 units | 78.00 units |
| Compute Time | 0.006 s | 0.009 s |
| Nodes Explored | 492 | 364 |

---

## 🗺️ Roadmap

- [ ] **RRT\*** — optimal rewiring variant with asymptotic convergence
- [ ] **GIF / video export** — save animations for presentations
- [ ] **Continuous space** — obstacle-free Voronoi-based environment
- [ ] **Online replanning** — dynamic obstacle avoidance during execution
- [ ] **Additional algorithms** — Dijkstra, PRM, Bi-directional RRT
- [ ] **3D extension** — extend to 3D workspace with quaternion rotations

---

## 📚 References

1. Hart, P. E., Nilsson, N. J. & Raphael, B. "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 1968.
2. LaValle, S. M. "Rapidly-Exploring Random Trees: A New Tool for Path Planning." *Technical Report TR 98-11*, Iowa State University, 1998.
3. Karaman, S. & Frazzoli, E. "Sampling-based Algorithms for Optimal Motion Planning." *International Journal of Robotics Research*, 30(7), 2011.
4. Liang, Y. D. & Barsky, B. A. "A New Concept and Method for Line Clipping." *ACM TOG*, 3(1), 1984.
5. LaValle, S. M. *Planning Algorithms*. Cambridge University Press, 2006. Available at http://planning.cs.uiuc.edu/

---

## 📄 License

MIT — free for personal and commercial use. Attribution appreciated.

---

## 🤖 AI Acknowledgment

This project was developed with the assistance of **Claude** (Anthropic). The AI helped with architecture design, code implementation, test creation, and documentation. All logic was reviewed and validated by the author to ensure correctness.

> I believe in transparency about the use of AI tools in software development.
