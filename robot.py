"""Robot controller for the 2D hospital navigation simulation."""

import sys
import time

sys.path.append(".")
from hospital_map import HospitalMap, locations, goals

sys.path.append("./algorithms")
from bfs import BFS, run_multi_goal_bfs
from astar import AStar, run_multi_goal_astar
from dijkstra import Dijkstra, run_multi_goal_dijkstra


class Robot:
    """Control the robot movement and mission execution on the hospital map."""

    def __init__(self, hospital_map, algorithm="astar"):
        """Store the map, select an algorithm, and initialize mission state."""
        self.hospital_map = hospital_map
        self.algorithm = ""
        self.current_position = hospital_map.get_start()
        self.mission_log = []
        self.total_steps = 0
        self.total_cost = 0
        self.status = "IDLE"
        self.mission_time = 0
        self.set_algorithm(algorithm)

    def set_algorithm(self, algorithm):
        """Switch between BFS and A* path planning."""
        normalized = algorithm.strip().lower()
        if normalized not in {"bfs", "astar", "dijkstra"}:
            raise ValueError("Algorithm must be 'bfs', 'astar', or 'dijkstra'.")

        self.algorithm = normalized
        print(f"Algorithm set to: {self._algorithm_label()}")

    def move_to(self, goal, goal_name="Unknown"):
        """Find a path to the goal and simulate movement step by step."""
        solver = self._create_solver()
        start_position = self.current_position
        from_name = self._find_location_name(start_position)
        self.status = "MOVING"

        path = solver.search(start_position, goal)
        if not path:
            self.status = "IDLE"
            print(f"No path found from {from_name} to {goal_name}.")
            return None

        # Count the first cell only once across the entire mission.
        segment_cost = self._calculate_segment_cost(path)
        segment_steps = max(len(path) - 1, 0)

        for step_number, next_position in enumerate(path[1:], start=1):
            print(f"Robot moving: {self.current_position} -> {next_position} [Step {step_number}]")
            self.current_position = next_position
            self.total_steps += 1
            self.total_cost += self.hospital_map.get_cost(*next_position)

        if not self.mission_log:
            self.total_cost += self.hospital_map.get_cost(*path[0])

        self.status = "ARRIVED"
        self.mission_log.append(
            {
                "from": from_name,
                "to": goal_name,
                "steps": segment_steps,
                "cost": segment_cost,
                "algorithm": self._algorithm_label(),
            }
        )

        return path

    def execute_mission(self):
        """Execute the full mission sequence from the Pharmacy and back."""
        self._reset_mission_state()
        print(f"Mission started at {self._find_location_name(self.current_position)} {self.current_position}")

        mission_start = time.perf_counter()
        for goal_name, goal_coordinates in goals:
            path = self.move_to(goal_coordinates, goal_name)
            if not path:
                print(f"Mission aborted before reaching {goal_name}.")
                return None

            mission_entry = self.mission_log[-1]
            print(
                f"ARRIVED at {goal_name} | Steps: {mission_entry['steps']} | "
                f"Cost: {mission_entry['cost']}"
            )

        self.mission_time = (time.perf_counter() - mission_start) * 1000
        self.status = "MISSION_COMPLETE"
        print(
            f"Mission complete | Total Steps: {self.total_steps} | "
            f"Total Cost: {self.total_cost} | Time: {self.mission_time:.2f} ms"
        )
        return self.mission_log

    def get_mission_report(self):
        """Return a formatted text report for the completed mission."""
        lines = [
            "=== ROBOT MISSION REPORT ===",
            f"Algorithm Used: {self._algorithm_label()}",
            f"Starting Position: Pharmacy {locations['Pharmacy']}",
            "",
            "Mission Log:",
        ]

        if not self.mission_log:
            lines.append("No missions completed yet.")
        else:
            for index, entry in enumerate(self.mission_log, start=1):
                route_label = f"{entry['from']} -> {entry['to']}"
                lines.append(
                    f"{index}. {route_label:<26} | "
                    f"Steps: {entry['steps']:<2} | Cost: {entry['cost']}"
                )

        status_label = self.status.replace("_", " ")
        if self.status == "MISSION_COMPLETE":
            status_label = "MISSION COMPLETE"

        lines.append("")
        lines.append(
            f"TOTAL: {self.total_steps} steps | Cost: {self.total_cost} | Status: {status_label}"
        )
        return "\n".join(lines)

    def compare_algorithms(self):
        """Run the full mission with BFS, Dijkstra, and A* and print a comparison."""
        bfs_path, bfs_stats = run_multi_goal_bfs(self.hospital_map)
        dijkstra_path, dijkstra_stats = run_multi_goal_dijkstra(self.hospital_map)
        astar_path, astar_stats = run_multi_goal_astar(self.hospital_map)

        bfs_steps = bfs_stats["path_length"] if bfs_path else 0
        dijkstra_steps = dijkstra_stats["path_length"] if dijkstra_path else 0
        astar_steps = astar_stats["path_length"] if astar_path else 0

        print("+------------------+----------+----------+----------+")
        print("| Metric           |   BFS    | Dijkstra |    A*    |")
        print("+------------------+----------+----------+----------+")
        print(f"| Total Steps      | {bfs_steps:>8} | {dijkstra_steps:>8} | {astar_steps:>8} |")
        print(
            f"| Total Cost       | {bfs_stats['path_cost']:>8} | "
            f"{dijkstra_stats['path_cost']:>8} | {astar_stats['path_cost']:>8} |"
        )
        print(
            f"| Execution Time   | {bfs_stats['execution_time']:>6.2f} ms | "
            f"{dijkstra_stats['execution_time']:>6.2f} ms | {astar_stats['execution_time']:>6.2f} ms |"
        )
        print("+------------------+----------+----------+----------+")

        candidates = [
            ("BFS", bfs_stats),
            ("Dijkstra", dijkstra_stats),
            ("A*", astar_stats),
        ]
        better_algorithm = min(
            candidates,
            key=lambda item: (item[1]["path_cost"], item[1]["execution_time"], item[1]["visited_nodes"]),
        )[0]

        print(f"Better algorithm for this mission: {better_algorithm}")
        return better_algorithm

    def _create_solver(self):
        """Create the selected pathfinding solver."""
        if self.algorithm == "bfs":
            return BFS(self.hospital_map)
        if self.algorithm == "dijkstra":
            return Dijkstra(self.hospital_map)
        return AStar(self.hospital_map)

    def _algorithm_label(self):
        """Return a display-friendly algorithm name."""
        if self.algorithm == "astar":
            return "A*"
        if self.algorithm == "dijkstra":
            return "Dijkstra"
        return "BFS"

    def _find_location_name(self, coordinates):
        """Resolve a coordinate to its known hospital location name."""
        for name, position in locations.items():
            if position == coordinates:
                return name
        return "Unknown"

    def _calculate_segment_cost(self, path):
        """Calculate segment cost while avoiding duplicate counting between legs."""
        if not path:
            return 0

        if not self.mission_log:
            return sum(self.hospital_map.get_cost(x, y) for x, y in path)
        return sum(self.hospital_map.get_cost(x, y) for x, y in path[1:])

    def _reset_mission_state(self):
        """Reset the robot so a fresh mission can be executed."""
        self.current_position = self.hospital_map.get_start()
        self.mission_log = []
        self.total_steps = 0
        self.total_cost = 0
        self.status = "IDLE"
        self.mission_time = 0


if __name__ == "__main__":
    hospital = HospitalMap()

    print("=== HOSPITAL ROBOT SIMULATION ===\n")

    # Test the full mission using A* as the default navigation algorithm.
    robot = Robot(hospital, algorithm="astar")
    robot.execute_mission()
    print(robot.get_mission_report())

    # Compare the full-mission performance of BFS and A*.
    print("\n=== ALGORITHM COMPARISON ===")
    robot.compare_algorithms()
