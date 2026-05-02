# 🗺️ Trajectory Planner — A* vs RRT

![Python](https://img.shields.io/badge/Python-3.10%2B-blue?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green)
![NumPy](https://img.shields.io/badge/NumPy-2.4-013243?logo=numpy)
![Matplotlib](https://img.shields.io/badge/Matplotlib-3.10-11557c)

An interactive 2D path-planning application that compares **A\*** (grid-based) and **RRT** (sampling-based) algorithms side by side. Built with Python, NumPy, Matplotlib, and Tkinter.

<p align="center">
  <img src="docs/demo.gif" alt="Demo" width="800">
  <br>
  <em>Side-by-side animated comparison of A* and RRT solving the same environment</em>
</p>

> **Note:** Demo GIF will be added once the application is fully functional.

---

## ✨ Features

- **Dual algorithm visualization** — watch A\* and RRT solve the same map simultaneously with step-by-step animation
- **A\* with configurable heuristics** — Manhattan, Euclidean, and Diagonal (Chebyshev) distance functions
- **RRT with tunable parameters** — step size, maximum iterations, goal bias, and goal threshold
- **Interactive environment editor** — draw and erase rectangular obstacles with the mouse, place start/goal with a click
- **Dynamic obstacles** — linear (bouncing) and circular (orbital) moving obstacles that update in real time
- **Map presets** — quickly load empty, maze, or random environments
- **Quantitative comparison panel** — side-by-side metrics: path length, computation time, and nodes explored
- **Clean, modular architecture** — designed for readability, extensibility, and portfolio presentation

---

## 🏗️ Architecture

```
trajectory_planner/
├── main.py                  # Application entry point
├── config.py                # Global constants and default parameters
├── requirements.txt         # Python dependencies
│
├── environment/             # Environment modeling layer
│   ├── __init__.py
│   ├── base_env.py          # Abstract base environment
│   ├── grid_env.py          # Grid-based environment with collision detection
│   ├── obstacle.py          # Static and dynamic obstacle classes
│   └── map_presets.py       # Predefined environment configurations
│
├── algorithms/              # Path planning algorithms
│   ├── __init__.py
│   ├── base_planner.py      # Abstract base planner interface
│   ├── astar.py             # A* search with multiple heuristics
│   └── rrt.py               # Rapidly-exploring Random Tree
│
├── metrics/                 # Performance evaluation
│   ├── __init__.py
│   └── evaluator.py         # Metrics collection and comparison
│
└── ui/                      # User interface layer
    ├── __init__.py
    ├── app.py               # Main application orchestrator
    ├── canvas_widget.py      # Matplotlib canvas with mouse interaction
    ├── control_panel.py      # Parameter controls and action buttons
    └── metrics_panel.py      # Comparison charts and tables
```

---

## 🚀 Installation

### Prerequisites

- Python 3.10 or higher
- Tkinter (included with standard Python installations)

### Setup

```bash
# Clone the repository
git clone https://github.com/<your-username>/trajectory_planner.git
cd trajectory_planner

# Create and activate virtual environment
python -m venv venv

# Windows
venv\Scripts\activate

# macOS / Linux
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

---

## 🎮 Usage

```bash
python main.py
```

### Controls

| Action | How |
|---|---|
| **Draw obstacle** | Select "Draw" mode → click and drag on canvas |
| **Erase obstacle** | Select "Erase" mode → click and drag on canvas |
| **Set start point** | Select "Set Start" mode → click on canvas |
| **Set goal point** | Select "Set Goal" mode → click on canvas |
| **Run algorithm** | Click "Run" or "Run Both" button |
| **Step through** | Click "Step" for one iteration at a time |
| **Pause / Resume** | Click "Pause" during animation |
| **Reset** | Click "Reset" to clear paths and restore environment |
| **Change speed** | Adjust the speed slider |
| **Load preset** | Select from the preset dropdown |

### Algorithm Parameters

**A\***
- Heuristic: Manhattan · Euclidean · Diagonal

**RRT**
- Step size: distance extended per iteration
- Max iterations: upper limit on tree expansion
- Goal bias: probability [0–1] of sampling toward the goal
- Goal threshold: distance to consider goal reached

---

## 📊 Metrics Comparison

The application computes and displays the following metrics for each algorithm:

| Metric | Description |
|---|---|
| **Path Length** | Total Euclidean distance of the solution path |
| **Compute Time** | Wall-clock time to find the solution (seconds) |
| **Nodes Explored** | Number of nodes expanded (A\*) or added to tree (RRT) |

Example comparison output:

| Metric | A* (Euclidean) | RRT |
|---|---|---|
| Path Length | 42.4 units | 51.7 units |
| Compute Time | 0.023 s | 0.041 s |
| Nodes Explored | 312 | 487 |

---

## 🗺️ Roadmap

- [ ] **RRT\*** — optimal rewiring variant with asymptotic convergence
- [ ] **GIF / video export** — save animations for presentations
- [ ] **Continuous space** — obstacle-free Voronoi-based environment
- [ ] **Online replanning** — dynamic obstacle avoidance during execution
- [ ] **Additional algorithms** — Dijkstra, PRM, Bi-directional RRT
- [ ] **3D extension** — extend to 3D workspace with quaternion rotations

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Language | Python 3.10+ |
| Numerical computing | NumPy |
| Visualization | Matplotlib |
| GUI framework | Tkinter (stdlib) |
| Version control | Git + GitHub |

---

## 📝 License

This project is licensed under the MIT License.

```
MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
