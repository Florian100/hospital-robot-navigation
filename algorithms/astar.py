"""A* search for 2D hospital robot navigation."""

import sys
import time
import heapq

sys.path.append("..")
from hospital_map import HospitalMap, locations, goals


def heuristic(a, b):
    """Estimate the remaining distance using Manhattan distance."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class AStar:
    """A* pathfinder that uses both distance and cell traversal cost."""

    def __init__(self, hospital_map):
        """Store the hospital map and initialize search statistics."""
        self.hospital_map = hospital_map
        self.steps = 0
        self.visited_nodes = 0
        self.execution_time = 0
        self.last_path = None
        self.last_visited_order = []

    def search(self, start, goal):
        """Run A* from start to goal using a priority queue."""
        start_time = time.perf_counter()
        open_set = []
        came_from = {}
        g_score = {start: 0}
        closed_set = set()
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        self.steps = 0
        self.visited_nodes = 0
        self.execution_time = 0
        self.last_path = None
        self.last_visited_order = []

        # Push the start node with f(n) = g(n) + h(n).
        heapq.heappush(open_set, (heuristic(start, goal), 0, start))

        while open_set:
            _, current_g, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)
            self.visited_nodes += 1
            self.last_visited_order.append(current)

            if current == goal:
                path = self.reconstruct_path(came_from, start, goal)
                self.last_path = path
                self.steps = max(len(path) - 1, 0)
                self.execution_time = (time.perf_counter() - start_time) * 1000
                return path

            current_x, current_y = current

            # Explore the four neighboring cells and skip blocked walls.
            for dx, dy in directions:
                next_x = current_x + dx
                next_y = current_y + dy
                neighbor = (next_x, next_y)

                if not (0 <= next_x < self.hospital_map.width and 0 <= next_y < self.hospital_map.height):
                    continue

                if self.hospital_map.get_cell(next_x, next_y) == 1:
                    continue

                if neighbor in closed_set:
                    continue

                movement_cost = self.hospital_map.get_cost(next_x, next_y)
                tentative_g = current_g + movement_cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        self.execution_time = (time.perf_counter() - start_time) * 1000
        return None

    def reconstruct_path(self, came_from, start, goal):
        """Rebuild the ordered path from goal back to start."""
        path = [goal]
        current = goal

        while current != start:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def get_stats(self):
        """Return a summary of the most recent A* search."""
        path_cost = 0
        if self.last_path:
            path_cost = sum(self.hospital_map.get_cost(x, y) for x, y in self.last_path)

        return {
            "algorithm": "A*",
            "path_length": self.steps,
            "visited_nodes": self.visited_nodes,
            "execution_time": self.execution_time,
            "path_cost": path_cost,
        }


def run_multi_goal_astar(hospital_map):
    """Run A* across the full mission sequence and combine the results."""
    astar = AStar(hospital_map)
    current_start = hospital_map.get_start()
    full_path = []
    unique_visited_nodes = set()
    total_execution_time = 0

    # Solve each leg of the hospital mission in sequence.
    for _, goal_coordinates in goals:
        segment_path = astar.search(current_start, goal_coordinates)
        if segment_path is None:
            return None, {
                "algorithm": "A*",
                "path_length": 0,
                "visited_nodes": len(unique_visited_nodes.union(astar.last_visited_order)),
                "execution_time": total_execution_time + astar.execution_time,
                "path_cost": 0,
            }

        if full_path:
            full_path.extend(segment_path[1:])
        else:
            full_path.extend(segment_path)

        segment_stats = astar.get_stats()
        unique_visited_nodes.update(astar.last_visited_order)
        total_execution_time += segment_stats["execution_time"]
        current_start = goal_coordinates

    total_path_cost = 0
    if full_path:
        total_path_cost = sum(hospital_map.get_cost(x, y) for x, y in full_path)

    total_stats = {
        "algorithm": "A*",
        "path_length": max(len(full_path) - 1, 0),
        "visited_nodes": len(unique_visited_nodes),
        "execution_time": total_execution_time,
        "path_cost": total_path_cost,
    }

    return full_path, total_stats


if __name__ == "__main__":
    # Create the hospital map and A* instance for the single-goal test.
    hospital = HospitalMap()
    astar = AStar(hospital)

    print("=== A* Single Goal Test ===")
    start = hospital.get_start()
    goal = hospital.get_goals()[0]
    path = astar.search(start, goal)

    if path:
        stats = astar.get_stats()
        print(f"Path found: {stats['path_length']} steps")
        print(f"Path: {path}")
        print(f"Visited nodes: {stats['visited_nodes']}")
        print(f"Execution time: {stats['execution_time']:.2f} ms")
        print(f"Path cost: {stats['path_cost']}")
    else:
        print("No path found!")
        stats = astar.get_stats()

    print("\n=== A* Multi-Goal Test ===")
    full_path, total_stats = run_multi_goal_astar(hospital)
    print(f"Total steps: {total_stats['path_length']}")
    print(f"Total cost: {total_stats['path_cost']}")
    print(f"Total time: {total_stats['execution_time']:.2f} ms")

    print("\n=== Comparison BFS vs A* ===")
    print("BFS - Steps: 19, Nodes: 109, Cost: 19, Time: 0.51ms")
    if path:
        print(
            f"A*  - Steps: {stats['path_length']}, Nodes: {stats['visited_nodes']}, "
            f"Cost: {stats['path_cost']}, Time: {stats['execution_time']:.2f}ms"
        )
