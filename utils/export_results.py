"""Export hospital navigation results to CSV and text summary files."""

import csv
import os
import sys
import datetime

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ALGORITHMS_DIR = os.path.join(BASE_DIR, "algorithms")
if BASE_DIR not in sys.path:
    sys.path.append(BASE_DIR)
if ALGORITHMS_DIR not in sys.path:
    sys.path.append(ALGORITHMS_DIR)

sys.path.append("..")
from hospital_map import HospitalMap

sys.path.append("../algorithms")
from bfs import BFS, run_multi_goal_bfs
from astar import AStar, run_multi_goal_astar
from dijkstra import Dijkstra, run_multi_goal_dijkstra


def export_single_goal_results(hospital):
    """Run all algorithms for the first goal and export the results to CSV."""
    start = hospital.get_start()
    goal = hospital.get_goals()[0]
    output_path = os.path.join(BASE_DIR, "results", "single_goal_results.csv")

    algorithms = [
        ("BFS", BFS(hospital)),
        ("Dijkstra", Dijkstra(hospital)),
        ("A*", AStar(hospital)),
    ]

    rows = []
    for algorithm_name, solver in algorithms:
        path = solver.search(start, goal)
        stats = solver.get_stats()
        rows.append(
            {
                "Algorithm": algorithm_name,
                "Steps": stats["path_length"] if path else 0,
                "Visited_Nodes": stats["visited_nodes"],
                "Path_Cost": stats["path_cost"],
                "Time_ms": f"{stats['execution_time']:.2f}",
            }
        )

    with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["Algorithm", "Steps", "Visited_Nodes", "Path_Cost", "Time_ms"],
        )
        writer.writeheader()
        writer.writerows(rows)

    return rows


def export_multi_goal_results(hospital):
    """Run all algorithms for the full mission and export the results to CSV."""
    output_path = os.path.join(BASE_DIR, "results", "multi_goal_results.csv")

    results = [
        ("BFS", run_multi_goal_bfs(hospital)[1]),
        ("Dijkstra", run_multi_goal_dijkstra(hospital)[1]),
        ("A*", run_multi_goal_astar(hospital)[1]),
    ]

    rows = []
    for algorithm_name, stats in results:
        rows.append(
            {
                "Algorithm": algorithm_name,
                "Total_Steps": stats["path_length"],
                "Total_Cost": stats["path_cost"],
                "Total_Time_ms": f"{stats['execution_time']:.2f}",
                "Visited_Nodes": stats["visited_nodes"],
            }
        )

    with open(output_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=["Algorithm", "Total_Steps", "Total_Cost", "Total_Time_ms", "Visited_Nodes"],
        )
        writer.writeheader()
        writer.writerows(rows)

    return rows


def export_all(hospital):
    """Create the results folder and export all supported result files."""
    results_dir = os.path.join(BASE_DIR, "results")
    os.makedirs(results_dir, exist_ok=True)

    single_rows = export_single_goal_results(hospital)
    multi_rows = export_multi_goal_results(hospital)

    winner = min(
        multi_rows,
        key=lambda row: (int(row["Total_Cost"]), float(row["Total_Time_ms"]), int(row["Visited_Nodes"])),
    )["Algorithm"]

    summary_path = os.path.join(results_dir, "summary.txt")
    generated_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    summary_lines = [
        "===================================",
        "HOSPITAL ROBOT NAVIGATION - RESULTS",
        f"Generated: {generated_time}",
        "===================================",
        "",
        "SINGLE GOAL (Pharmacy -> Room 101):",
        "Algorithm  | Steps | Nodes | Cost | Time",
    ]

    for row in single_rows:
        summary_lines.append(
            f"{row['Algorithm']:<10} | {int(row['Steps']):>5} | {int(row['Visited_Nodes']):>5} | "
            f"{int(row['Path_Cost']):>4} | {float(row['Time_ms']):>6.2f} ms"
        )

    summary_lines.extend(
        [
            "",
            "MULTI-GOAL (Full Mission):",
            "Algorithm  | Steps | Cost | Time",
        ]
    )

    for row in multi_rows:
        summary_lines.append(
            f"{row['Algorithm']:<10} | {int(row['Total_Steps']):>5} | {int(row['Total_Cost']):>4} | "
            f"{float(row['Total_Time_ms']):>6.2f} ms"
        )

    summary_lines.extend(
        [
            "",
            f"WINNER: {winner} (lowest cost + fewest nodes)",
            "===================================",
        ]
    )

    with open(summary_path, "w", encoding="utf-8") as summary_file:
        summary_file.write("\n".join(summary_lines))


if __name__ == "__main__":
    hospital = HospitalMap()
    export_all(hospital)
    print("Results exported to results/ folder!")
    print("Files: single_goal_results.csv")
    print("       multi_goal_results.csv")
    print("       summary.txt")
