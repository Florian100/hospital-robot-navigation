"""Breadth-First Search for 2D hospital robot navigation."""

import sys
import time
from collections import deque

sys.path.append("..")
from hospital_map import HospitalMap, locations, goals


class BFS:
    """Standard BFS pathfinder that treats all passable cells equally."""

    def __init__(self, hospital_map):
        """Store the hospital map and initialize search statistics."""
        self.hospital_map = hospital_map
        self.steps = 0
        self.visited_nodes = 0
        self.execution_time = 0
        self.last_path = None
        self.last_visited_order = []

    def search(self, start, goal):
        """Run BFS from start to goal using four-direction movement."""
        start_time = time.perf_counter()
        queue = deque([start])
        came_from = {}
        visited = {start}
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        self.steps = 0
        self.visited_nodes = 0
        self.execution_time = 0
        self.last_path = None
        self.last_visited_order = []

        while queue:
            current = queue.popleft()
            self.visited_nodes += 1
            self.last_visited_order.append(current)

            if current == goal:
                path = self.reconstruct_path(came_from, start, goal)
                self.last_path = path
                self.steps = max(len(path) - 1, 0)
                self.execution_time = (time.perf_counter() - start_time) * 1000
                return path

            current_x, current_y = current

            # Explore neighbors in four directions and ignore walls.
            for dx, dy in directions:
                next_x = current_x + dx
                next_y = current_y + dy
                next_cell = (next_x, next_y)

                if not (0 <= next_x < self.hospital_map.width and 0 <= next_y < self.hospital_map.height):
                    continue

                if next_cell in visited:
                    continue

                if self.hospital_map.get_cell(next_x, next_y) == 1:
                    continue

                visited.add(next_cell)
                came_from[next_cell] = current
                queue.append(next_cell)

        self.execution_time = (time.perf_counter() - start_time) * 1000
        return None

    def reconstruct_path(self, came_from, start, goal):
        """Rebuild the ordered path from the goal back to the start."""
        path = [goal]
        current = goal

        while current != start:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def get_stats(self):
        """Return a summary of the latest search statistics."""
        path_cost = 0
        if self.last_path:
            path_cost = sum(self.hospital_map.get_cost(x, y) for x, y in self.last_path)

        return {
            "algorithm": "BFS",
            "path_length": self.steps,
            "visited_nodes": self.visited_nodes,
            "execution_time": self.execution_time,
            "path_cost": path_cost,
        }


def run_multi_goal_bfs(hospital_map):
    """Run BFS across the full mission sequence and combine all results."""
    bfs = BFS(hospital_map)
    current_start = hospital_map.get_start()
    full_path = []
    unique_visited_nodes = set()
    total_execution_time = 0
    total_path_cost = 0

    # Visit each mission goal sequentially and merge the resulting paths.
    for _, goal_coordinates in goals:
        segment_path = bfs.search(current_start, goal_coordinates)
        if segment_path is None:
            return None, {
                "algorithm": "BFS",
                "path_length": 0,
                "visited_nodes": len(unique_visited_nodes.union(bfs.last_visited_order)),
                "execution_time": total_execution_time + bfs.execution_time,
                "path_cost": total_path_cost,
            }

        if full_path:
            full_path.extend(segment_path[1:])
        else:
            full_path.extend(segment_path)

        segment_stats = bfs.get_stats()
        unique_visited_nodes.update(bfs.last_visited_order)
        total_execution_time += segment_stats["execution_time"]
        total_path_cost += segment_stats["path_cost"]
        current_start = goal_coordinates

    total_stats = {
        "algorithm": "BFS",
        "path_length": max(len(full_path) - 1, 0),
        "visited_nodes": len(unique_visited_nodes),
        "execution_time": total_execution_time,
        "path_cost": total_path_cost,
    }

    return full_path, total_stats


if __name__ == "__main__":
    # Create the hospital map and BFS instance for a single-goal test.
    hospital = HospitalMap()
    bfs = BFS(hospital)

    print("=== BFS Single Goal Test ===")
    start = hospital.get_start()
    goal = hospital.get_goals()[0]
    path = bfs.search(start, goal)

    if path:
        stats = bfs.get_stats()
        print(f"Path found: {stats['path_length']} steps")
        print(f"Path: {path}")
        print(f"Visited nodes: {stats['visited_nodes']}")
        print(f"Execution time: {stats['execution_time']:.2f} ms")
        print(f"Path cost: {stats['path_cost']}")
    else:
        print("No path found!")

    print("\n=== BFS Multi-Goal Test ===")
    full_path, total_stats = run_multi_goal_bfs(hospital)
    print(f"Total steps: {total_stats['path_length']}")
    print(f"Total cost: {total_stats['path_cost']}")
    print(f"Total time: {total_stats['execution_time']:.2f} ms")
