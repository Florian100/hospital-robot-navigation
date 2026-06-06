"""Compare BFS and A* for the hospital robot navigation simulation."""

import sys
import time
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None
import numpy as np

sys.path.append("..")

BASE_DIR = Path(__file__).resolve().parent.parent
if str(BASE_DIR) not in sys.path:
    sys.path.append(str(BASE_DIR))

from hospital_map import HospitalMap

ALGORITHMS_DIR = BASE_DIR / "algorithms"
if str(ALGORITHMS_DIR) not in sys.path:
    sys.path.append(str(ALGORITHMS_DIR))

from bfs import BFS, run_multi_goal_bfs
from dijkstra import Dijkstra, run_multi_goal_dijkstra
from astar import AStar, run_multi_goal_astar


class AlgorithmComparator:
    """Run and compare BFS and A* on the hospital map."""

    def __init__(self, hospital_map):
        """Store the shared map and create reusable algorithm instances."""
        self.hospital_map = hospital_map
        self.bfs = BFS(hospital_map)
        self.dijkstra = Dijkstra(hospital_map)
        self.astar = AStar(hospital_map)

    def run_single_goal_comparison(self):
        """Compare BFS and A* on the Pharmacy to Room_101 route."""
        start = self.hospital_map.get_start()
        goal = self.hospital_map.get_goals()[0]

        bfs_path = self.bfs.search(start, goal)
        bfs_stats = self.bfs.get_stats()

        dijkstra_path = self.dijkstra.search(start, goal)
        dijkstra_stats = self.dijkstra.get_stats()

        astar_path = self.astar.search(start, goal)
        astar_stats = self.astar.get_stats()

        return {
            "bfs": {
                "path": bfs_path,
                "steps": bfs_stats["path_length"],
                "visited_nodes": bfs_stats["visited_nodes"],
                "path_cost": bfs_stats["path_cost"],
                "execution_time": bfs_stats["execution_time"],
            },
            "dijkstra": {
                "path": dijkstra_path,
                "steps": dijkstra_stats["path_length"],
                "visited_nodes": dijkstra_stats["visited_nodes"],
                "path_cost": dijkstra_stats["path_cost"],
                "execution_time": dijkstra_stats["execution_time"],
            },
            "astar": {
                "path": astar_path,
                "steps": astar_stats["path_length"],
                "visited_nodes": astar_stats["visited_nodes"],
                "path_cost": astar_stats["path_cost"],
                "execution_time": astar_stats["execution_time"],
            },
        }

    def run_multi_goal_comparison(self):
        """Compare BFS and A* across the full multi-goal mission."""
        bfs_path, bfs_stats = run_multi_goal_bfs(self.hospital_map)
        dijkstra_path, dijkstra_stats = run_multi_goal_dijkstra(self.hospital_map)
        astar_path, astar_stats = run_multi_goal_astar(self.hospital_map)

        return {
            "bfs": {
                "path": bfs_path,
                "steps": bfs_stats["path_length"],
                "path_cost": bfs_stats["path_cost"],
                "execution_time": bfs_stats["execution_time"],
            },
            "dijkstra": {
                "path": dijkstra_path,
                "steps": dijkstra_stats["path_length"],
                "path_cost": dijkstra_stats["path_cost"],
                "execution_time": dijkstra_stats["execution_time"],
                "visited_nodes": dijkstra_stats["visited_nodes"],
            },
            "astar": {
                "path": astar_path,
                "steps": astar_stats["path_length"],
                "path_cost": astar_stats["path_cost"],
                "execution_time": astar_stats["execution_time"],
                "visited_nodes": astar_stats["visited_nodes"],
            },
        }

    def print_comparison_table(self, single_results, multi_results):
        """Print formatted comparison tables and a short winner summary."""
        print("=== SINGLE GOAL COMPARISON (Pharmacy -> Room 101) ===")
        print("+------------------+----------+----------+----------+")
        print("| Metric           |   BFS    | Dijkstra |    A*    |")
        print("+------------------+----------+----------+----------+")
        print(
            f"| Steps            |"
            f" {single_results['bfs']['steps']:>8} |"
            f" {single_results['dijkstra']['steps']:>8} |"
            f" {single_results['astar']['steps']:>8} |"
        )
        print(
            f"| Visited Nodes    |"
            f" {single_results['bfs']['visited_nodes']:>8} |"
            f" {single_results['dijkstra']['visited_nodes']:>8} |"
            f" {single_results['astar']['visited_nodes']:>8} |"
        )
        print(
            f"| Path Cost        |"
            f" {single_results['bfs']['path_cost']:>8} |"
            f" {single_results['dijkstra']['path_cost']:>8} |"
            f" {single_results['astar']['path_cost']:>8} |"
        )
        print(
            f"| Execution Time   |"
            f" {single_results['bfs']['execution_time']:>6.2f} ms |"
            f" {single_results['dijkstra']['execution_time']:>6.2f} ms |"
            f" {single_results['astar']['execution_time']:>6.2f} ms |"
        )
        print("+------------------+----------+----------+----------+")

        print("\n=== MULTI-GOAL COMPARISON (Full Mission) ===")
        print("+------------------+----------+----------+----------+")
        print("| Metric           |   BFS    | Dijkstra |    A*    |")
        print("+------------------+----------+----------+----------+")
        print(
            f"| Total Steps      |"
            f" {multi_results['bfs']['steps']:>8} |"
            f" {multi_results['dijkstra']['steps']:>8} |"
            f" {multi_results['astar']['steps']:>8} |"
        )
        print(
            f"| Total Cost       |"
            f" {multi_results['bfs']['path_cost']:>8} |"
            f" {multi_results['dijkstra']['path_cost']:>8} |"
            f" {multi_results['astar']['path_cost']:>8} |"
        )
        print(
            f"| Total Time       |"
            f" {multi_results['bfs']['execution_time']:>6.2f} ms |"
            f" {multi_results['dijkstra']['execution_time']:>6.2f} ms |"
            f" {multi_results['astar']['execution_time']:>6.2f} ms |"
        )
        print("+------------------+----------+----------+----------+")

        single_candidates = [
            ("BFS", single_results["bfs"]),
            ("Dijkstra", single_results["dijkstra"]),
            ("A*", single_results["astar"]),
        ]
        multi_candidates = [
            ("BFS", multi_results["bfs"]),
            ("Dijkstra", multi_results["dijkstra"]),
            ("A*", multi_results["astar"]),
        ]
        best_nodes_name, best_nodes_stats = min(
            single_candidates,
            key=lambda item: (item[1]["visited_nodes"], item[1]["execution_time"]),
        )
        best_cost_name, best_cost_stats = min(
            multi_candidates,
            key=lambda item: (item[1]["path_cost"], item[1]["execution_time"]),
        )
        best_speed_name, best_speed_stats = min(
            multi_candidates,
            key=lambda item: item[1]["execution_time"],
        )

        print("\n=== WINNER ===")
        print(
            f"Visited Nodes: {best_nodes_name} wins "
            f"({best_nodes_stats['visited_nodes']} nodes)"
        )
        print(
            f"Path Cost:     {best_cost_name} wins "
            f"({best_cost_stats['path_cost']} total cost)"
        )
        print(
            f"Speed:         {best_speed_name} wins "
            f"({best_speed_stats['execution_time']:.2f} ms)"
        )

    def generate_comparison_charts(self, single_results, multi_results):
        """Generate bar charts for the key BFS versus A* metrics."""
        if plt is None:
            raise ImportError("matplotlib is required to generate comparison charts.")

        labels = ["BFS", "Dijkstra", "A*"]
        colors = ["#E74C3C", "#F39C12", "#2E86AB"]
        x_positions = np.arange(len(labels))

        single_nodes = [
            single_results["bfs"]["visited_nodes"],
            single_results["dijkstra"]["visited_nodes"],
            single_results["astar"]["visited_nodes"],
        ]
        multi_costs = [
            multi_results["bfs"]["path_cost"],
            multi_results["dijkstra"]["path_cost"],
            multi_results["astar"]["path_cost"],
        ]
        multi_times = [
            multi_results["bfs"]["execution_time"],
            multi_results["dijkstra"]["execution_time"],
            multi_results["astar"]["execution_time"],
        ]

        # Build three side-by-side charts for the requested metrics.
        figure, axes = plt.subplots(1, 3, figsize=(15, 5))
        figure.suptitle("BFS vs Dijkstra vs A* Comparison", fontsize=14, fontweight="bold")

        axes[0].bar(x_positions, single_nodes, color=colors, width=0.6)
        axes[0].set_title("Visited Nodes")
        axes[0].set_ylabel("Nodes")
        axes[0].set_xticks(x_positions, labels)

        axes[1].bar(x_positions, multi_costs, color=colors, width=0.6)
        axes[1].set_title("Multi-Goal Path Cost")
        axes[1].set_ylabel("Cost")
        axes[1].set_xticks(x_positions, labels)

        axes[2].bar(x_positions, multi_times, color=colors, width=0.6)
        axes[2].set_title("Multi-Goal Execution Time")
        axes[2].set_ylabel("Time (ms)")
        axes[2].set_xticks(x_positions, labels)

        for axis, values in zip(axes, [single_nodes, multi_costs, multi_times]):
            for index, value in enumerate(values):
                offset = max(value * 0.02, 0.05)
                axis.text(index, value + offset, f"{value:.2f}" if isinstance(value, float) else str(value), ha="center")

        figure.tight_layout()
        output_path = Path(__file__).resolve().parent / "comparison_results.png"
        plt.savefig(output_path, dpi=200, bbox_inches="tight")
        plt.show()


if __name__ == "__main__":
    # Create the map and comparator, then run both experiments.
    hospital = HospitalMap()
    comparator = AlgorithmComparator(hospital)

    print("Running comparisons...")
    start_time = time.perf_counter()
    single = comparator.run_single_goal_comparison()
    multi = comparator.run_multi_goal_comparison()

    comparator.print_comparison_table(single, multi)
    comparator.generate_comparison_charts(single, multi)

    total_runtime = (time.perf_counter() - start_time) * 1000
    print("\nChart saved as 'comparison_results.png'")
    print(f"Comparison runtime: {total_runtime:.2f} ms")
