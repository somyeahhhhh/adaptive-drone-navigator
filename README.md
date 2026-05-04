# 🚁 Adaptive Drone Navigation System
### Dynamic A* Replanning under Uncertain Intelligence

> **B.Tech CSE — Data Structures & Algorithms / Competitive Programming Lab Project**

---

## 📌 Project Description

This project simulates an autonomous rescue drone navigating a partially-known grid environment. The drone uses the **A\* search algorithm** to find the shortest path to its goal, but unlike a standard A\* implementation, it operates with **incomplete information** — some obstacles are hidden and only revealed when the drone's sensor comes within range.

When the drone discovers a hidden obstacle that blocks its current path, it **dynamically replans** using A\* on the updated map. This introduces a measurable **"Cost of Uncertainty"** — the extra steps taken compared to a drone with perfect knowledge of the environment.

The project includes:
- A fully-commented **C++ simulation** with step-by-step terminal output and ANSI colors
- An **interactive web visualizer** (HTML/JS) showing the drone navigating the grid in real time

---

## 🧠 Key Concepts & Algorithms

| Concept | Details |
|---|---|
| **A\* Search** | Finds optimal path using `f(n) = g(n) + h(n)` where `h` is Manhattan distance heuristic |
| **Min-Heap Priority Queue** | `std::priority_queue` used to always expand the lowest-cost node first |
| **Dynamic Replanning** | A\* is re-run every time a hidden obstacle invalidates the current path |
| **Sensor Simulation** | Drone has a Manhattan-radius sensor that reveals hidden obstacles nearby |
| **Cost of Uncertainty** | Quantifies the overhead of incomplete map knowledge vs. perfect information |

---

## 📁 Project Structure

```
CP_project/
├── drone_navigation.cpp         # Main C++ simulation (fully commented)
├── drone_navigation.exe         # Compiled Windows binary
├── drone_astar_explainer.html   # Interactive web visualizer
└── .vscode/
    ├── c_cpp_properties.json
    ├── launch.json
    └── settings.json
```

---

## Getting Started

### Prerequisites
- GCC / MinGW with C++17 support
- Windows Terminal (recommended for ANSI color support)

### Compile
```bash
g++ -std=c++17 -O2 -o drone drone_navigation.cpp
```

### Run
```bash
./drone
```

> **Windows encoding fix** — if you see a UnicodeEncodeError, run with:
> ```powershell
> $env:PYTHONUTF8=1; ./drone
> ```

### Web Visualizer
Open `drone_astar_explainer.html` directly in any modern browser. No server needed.

---

## 🗺️ Grid Scenario

```
   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
0  S  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
1  .  #  #  #  .  .  .  .  .  .  .  .  .  .  .  .
2  .  .  .  .  .  #  .  .  ?  ?  .  .  .  .  .  .
3  .  .  .  .  .  #  .  .  ?  ?  .  .  .  .  .  .
4  .  .  .  .  .  #  .  .  ?  ?  .  .  .  .  .  .
5  .  .  .  .  .  .  .  .  ?  ?  .  .  .  .  .  .
6  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
7  .  #  #  #  #  #  #  .  .  .  .  .  .  .  .  .
8  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
9  .  .  .  .  .  .  .  .  .  .  #  #  .  .  .  .
10 .  .  .  .  .  .  .  .  .  .  #  #  .  .  .  .
11 .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  G
```

`S` = Start (0,0) &nbsp; `G` = Goal (11,15) &nbsp; `#` = Known obstacle &nbsp; `?` = Hidden obstacle &nbsp; `.` = Free cell

The drone initially plans through column 8 (looks shortest). The hidden wall at rows 2–5, cols 8–9 forces a backtrack and replan — demonstrating the Cost of Uncertainty in action.

---

## 📊 Time Complexity

| Operation | Complexity |
|---|---|
| Single A\* run | O(N log N), N = rows × cols |
| k replans total | O(k × N log N) |
| Sensor scan per step | O(r²), r = sensor radius |
| **Worst case overall** | **O(N² log N)** |
| Space | O(N) |

For the 12×16 grid: N = 192 cells.

---

## 🏗️ Code Architecture

```
drone_navigation.cpp
├── struct Position            — (row, col) coordinate pair
├── struct Node                — A* node: f, g, h values + parent
├── class Grid                 — true_map vs known_map separation
├── class AStar                — Static A* with path reconstruction
├── class Drone                — Agent: position, sensor, step counter
├── class Visualizer           — ANSI ASCII grid renderer
├── runMission()               — Main loop: move → scan → replan
├── runPerfectInfoComparison() — Baseline with full map knowledge
├── printReport()              — Cost of Uncertainty metrics
└── printComplexityAnalysis()  — Complexity breakdown
```

---

## 📈 Sample Output

```
MISSION METRICS — DYNAMIC MODE
  Total steps taken    : 22
  Number of replans    : 3
  Actual path cost     : 22
  Optimal cost (perf.) : 19
  Cost of Uncertainty  : 3
  Overhead (%)         : 15.78%

CONCLUSION: Incomplete knowledge caused 3 extra step(s).
This is the measurable 'Cost of Uncertainty'.
```

---

## 💡 Design Highlights

- **Two-layer map design** — Separating `true_map` from `known_map` cleanly models partial observability
- **Admissible heuristic** — Manhattan distance never overestimates cost, guaranteeing optimality on the known map
- **Quantified uncertainty** — The Cost of Uncertainty metric gives a concrete, measurable result beyond just "the drone reached the goal"
- **Zero-dependency visualizer** — Plain HTML/JS, no frameworks, fully self-contained

---

## 👩‍💻 Author

**Somya** — B.Tech CSE, 2nd Year  
*Data Structures & Algorithms / Competitive Programming Lab*
