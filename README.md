


https://github.com/user-attachments/assets/20f5cc84-c37b-4785-b0c3-5e9d9faf1702

# Dynamic Pathfinding Agent 🗺️

> A professional grid-based pathfinding visualiser built with Python + Pygame,  
> implementing **A\*** and **Greedy Best-First Search** with real-time dynamic replanning.

---

## 📖 Description

This project simulates an intelligent agent navigating a grid map from a **Start** cell  
to a **Goal** cell using two informed search algorithms. The agent visualises every step  
of the search — frontier expansion, visited cells, and the final path — in a clean,  
animated GUI. A unique **Dynamic Mode** allows new obstacles to spawn while the agent  
is walking its path, forcing it to detect blockages and instantly replan its route.

The project was developed as a university assignment for the Artificial Intelligence  
course, demonstrating the practical application of heuristic search in dynamic environments.

---

## ✨ Features

| Feature | Details |
|---|---|
| **Grid configurator** | Set rows, columns, and obstacle density before launching |
| **A\* Search** | Optimal pathfinding using f(n) = g(n) + h(n) |
| **Greedy BFS** | Fast heuristic-only search using f(n) = h(n) |
| **Manhattan heuristic** | Best for 4-directional movement |
| **Euclidean heuristic** | Straight-line distance estimate |
| **Interactive editor** | Paint/erase walls with left/right mouse click |
| **Random map generator** | One-click randomised obstacle layouts |
| **Step-by-step animation** | Frontier (amber) and visited (red) cells animate live |
| **Agent walking** | White dot walks the final path after search completes |
| **Dynamic Mode** | ~4% chance per step to spawn a new obstacle; replans if blocked |
| **Metrics dashboard** | Nodes visited, path cost, execution time, algorithm, heuristic |
| **Clean window exit** | Clicking X or pressing ESC closes the app immediately |

---

## 🛠️ Installation

```bash
# Install the only required dependency
pip install pygame
```

> Requires Python 3.8 or higher.

---

## 🚀 How to Run

```bash
python pathfinding_agent.py
```

1. A **setup screen** appears — enter rows, columns, and obstacle percentage.
2. Click **START** to open the main visualiser.
3. Use the control panel on the right to configure and run the search.

---

## 🎮 Controls Guide

| Action | Input |
|---|---|
| Place wall | Left-click (or hold and drag) |
| Remove wall | Right-click (or hold and drag) |
| Start search | Click **Play Search** or press **Enter** |
| Reset simulation | Click **Reset** or press **R** |
| Quit application | Click window X or press **Escape** |
| Generate random map | Click **Generate Map** |
| Clear all walls | Click **Clear Walls** |
| Toggle algorithm | Click **Algorithm: A\*** or **Algorithm: GBFS** |
| Toggle heuristic | Click **Heuristic: Manhattan** or **Heuristic: Euclidean** |
| Toggle dynamic mode | Click **Dynamic Mode: OFF/ON** |

---

## 🔬 Algorithm Explanation

### A\* Search
A\* evaluates each candidate node using:

```
f(n) = g(n) + h(n)
```

- `g(n)` — the actual cost of the path from the start to node `n` (step count).
- `h(n)` — the heuristic estimate of remaining cost from `n` to the goal.
- `f(n)` — the total estimated cost; the priority queue is ordered by this value.

A\* is **complete** (always finds a path if one exists) and **optimal** (finds the  
shortest path), provided the heuristic is admissible — i.e., it never overestimates  
the true remaining cost. Both Manhattan and Euclidean satisfy this for 4-directional grids.

### Greedy Best-First Search (GBFS)
GBFS simplifies A\* by ignoring `g(n)` entirely:

```
f(n) = h(n)
```

It always expands the node that _looks_ closest to the goal. This makes it  
considerably faster in open environments but sacrifices optimality guarantees.  
In maze-like layouts, GBFS can produce paths much longer than the shortest route.

### Heuristics

| Heuristic | Formula | Notes |
|---|---|---|
| Manhattan | `|r1-r2| + |c1-c2|` | Exact lower bound for 4-dir grids |
| Euclidean | `sqrt((r1-r2)^2 + (c1-c2)^2)` | Slightly underestimates; more natural |

---

## ⚡ Dynamic Mode Explanation

When **Dynamic Mode** is enabled, the simulation introduces real-world uncertainty:

1. At each agent movement step, there is a ~4% probability that a new obstacle  
   will be placed somewhere on the grid (never on the start, goal, or agent cell).
2. After each new obstacle, the agent's remaining path is scanned for blockages.
3. If a blockage is detected, the algorithm is **re-run immediately** from the  
   agent's current position — without resetting the entire grid or restarting.
4. The agent then continues along the new route.
5. If no route exists after replanning, the simulation halts with "NO_PATH" status.

This mirrors real-world scenarios in robotics and autonomous navigation where  
the environment is partially unknown or changes over time.

---

## 🎨 Colour Reference

| Colour | Cell Type |
|---|---|
| Emerald green | Start node |
| Royal blue | Goal node |
| Dark charcoal | Obstacle / wall |
| Amber | Frontier (open list) |
| Muted red | Visited (closed list) |
| Bright lime | Final path |
| White dot | Moving agent |

---

## 📦 Dependencies

```
Python >= 3.8
pygame >= 2.0
```

Install with:
```bash
pip install pygame
```

---

## 📂 Project Structure

```
pathfinding_agent.py      ← Complete source code (single file)
README.md                 ← This documentation file
report.pdf                ← Academic report
```

---

## 📋 GitHub Commit Plan

```bash
git init
git add README.md
git commit -m "docs: add project README with setup and controls guide"

git add pathfinding_agent.py
git commit -m "feat: add Node and Grid classes with wall management"

git commit -m "feat: implement A* search with heapq priority queue"
git commit -m "feat: implement Greedy Best-First Search"
git commit -m "feat: add Manhattan and Euclidean heuristic functions"
git commit -m "feat: build setup screen with grid configuration dialog"
git commit -m "feat: add Pygame window, grid renderer, and colour theme"
git commit -m "feat: implement control panel with toggle buttons"
git commit -m "feat: add step-by-step search animation system"
git commit -m "feat: implement agent walking animation on final path"
git commit -m "feat: add dynamic obstacle spawning with path blockage detection"
git commit -m "feat: implement real-time path replanning from agent position"
git commit -m "feat: add metrics dashboard (nodes, cost, time)"
git commit -m "fix: ensure QUIT event handled in every loop iteration"
git commit -m "refactor: clean up comments and code structure for submission"
git commit -m "docs: add academic report PDF"
```

---

## 👨‍🎓 Academic Context

Developed for the **Artificial Intelligence** course — demonstrating informed  
search algorithms (A\* and GBFS) with a dynamic environment extension.


🔗 Links
📝 Medium Blog Post: https://medium.com/@f243019/i-built-an-ai-agent-that-finds-its-way-even-when-the-road-disappears-a1bc8d37d3b4
💼 LinkedIn: https://www.linkedin.com/in/zohashraf/
