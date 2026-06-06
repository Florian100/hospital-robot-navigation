# Hospital Robot Navigation Simulator

2D hospital robot navigation project built for **Elemente te Robotikes dhe Automatizimit**.

The project simulates a delivery robot moving inside a hospital on a `20x20` grid. It supports multiple pathfinding algorithms, terminal-based mission execution, algorithm comparison with charts, CSV export of results, and a visual simulator built with `Pygame`.

## Features

- `20x20` hospital grid with corridors, walls, and high-traffic zones
- Named hospital locations such as Pharmacy, Laboratory, ICU, ER, and patient rooms
- Pathfinding with:
  - `BFS`
  - `Dijkstra`
  - `A*`
- Multi-goal mission execution:
  - `Pharmacy -> Room_101 -> Laboratory -> ICU -> Pharmacy`
- Terminal robot simulation with step-by-step movement logs
- Visual simulation using `Pygame`
- Algorithm comparison with charts
- Export of results to CSV and summary text files

## Grid Model

The hospital map uses these cell values:

- `0` = free corridor, movement cost `1`
- `1` = wall / obstacle, impassable
- `2` = high-traffic zone, movement cost `5`

## Project Structure

```text
hospital-robot-navigation/
|-- algorithms/
|   |-- bfs.py
|   |-- dijkstra.py
|   `-- astar.py
|-- results/
|   |-- single_goal_results.csv
|   |-- multi_goal_results.csv
|   `-- summary.txt
|-- utils/
|   |-- comparator.py
|   |-- comparison_results.png
|   `-- export_results.py
|-- visualization/
|   `-- display.py
|-- hospital_map.py
|-- robot.py
|-- main.py
`-- README.md
```

## Requirements

Python `3.10+` is recommended.

Install the required packages:

```bash
pip install pygame matplotlib numpy
```

## Main Files

- [hospital_map.py](hospital_map.py)
  Defines the hospital grid, named locations, movement costs, and printable terminal map.

- [robot.py](robot.py)
  Controls robot missions, movement, statistics, and reports.

- [main.py](main.py)
  Entry point with the main menu for all project features.

## How to Run

### 1. Launch the main menu

```bash
python main.py
```

Menu options:

- `1` Run terminal simulation with `A*`
- `2` Run terminal simulation with `BFS`
- `3` Run terminal simulation with `Dijkstra`
- `4` Compare all algorithms and generate charts
- `5` Launch visual simulator with `Pygame`
- `6` Export results to CSV
- `7` Show hospital map
- `0` Exit

### 2. Run algorithms individually

From the `algorithms` folder:

```bash
python bfs.py
python dijkstra.py
python astar.py
```

### 3. Run comparison

From the project root:

```bash
python utils/export_results.py
python utils/comparator.py
```

### 4. Run visual simulator

From the project root:

```bash
python visualization/display.py
```

Controls in the visual simulator:

- `SPACE` Run or pause
- `B` Switch to `BFS`
- `A` Switch to `A*`
- `D` Switch to `Dijkstra`
- `R` Reset
- `V` Toggle visited cells
- `Q` or `ESC` Quit

## Algorithms

### BFS

- Ignores movement cost
- Treats all passable cells equally
- Good baseline for shortest path by step count only

### Dijkstra

- Uses real movement costs
- No heuristic
- Finds lowest-cost path, but typically explores more nodes than `A*`

### A*

- Uses real movement costs
- Uses Manhattan distance heuristic
- Usually the best balance between path quality and search efficiency

## Output Files

The export utility creates:

- [single_goal_results.csv](results/single_goal_results.csv)
- [multi_goal_results.csv](results/multi_goal_results.csv)
- [summary.txt](results/summary.txt)
- [comparison_results.png](utils/comparison_results.png)

## Mission Scenario

Default mission order:

1. Start at `Pharmacy`
2. Go to `Room_101`
3. Go to `Laboratory`
4. Go to `ICU`
5. Return to `Pharmacy`

## Notes

- `BFS` may find a path with the same number of steps but a higher total cost because it does not consider traffic cost.
- `Dijkstra` and `A*` both consider real movement cost.
- `A*` is typically the most efficient algorithm in this project.

## Author / Course

Project for **Elemente te Robotikes dhe Automatizimit**.
