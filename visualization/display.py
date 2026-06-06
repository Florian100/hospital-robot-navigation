"""Pygame visualization for the hospital robot navigation simulator."""

import sys
import pygame
import time
import heapq
from pathlib import Path
from collections import deque

BASE_DIR = Path(__file__).resolve().parent.parent
ALGORITHMS_DIR = BASE_DIR / "algorithms"
if str(BASE_DIR) not in sys.path:
    sys.path.append(str(BASE_DIR))
if str(ALGORITHMS_DIR) not in sys.path:
    sys.path.append(str(ALGORITHMS_DIR))

sys.path.append("..")
from hospital_map import HospitalMap, locations, goals

sys.path.append("../algorithms")
from bfs import BFS, run_multi_goal_bfs
from astar import AStar, run_multi_goal_astar
from dijkstra import Dijkstra, run_multi_goal_dijkstra

sys.path.append("..")
from robot import Robot


CELL_SIZE = 36
PANEL_WIDTH = 300
WINDOW_WIDTH = 20 * CELL_SIZE + PANEL_WIDTH
WINDOW_HEIGHT = 20 * CELL_SIZE
FPS = 10

WHITE = (255, 255, 255)
BLACK = (20, 20, 20)
WALL = (44, 62, 80)
CORRIDOR = (236, 240, 241)
HIGH_COST = (230, 126, 34)
PATH_BFS = (231, 76, 60)
PATH_ASTAR = (46, 204, 113)
PATH_DIJKSTRA = (241, 196, 15)
ROBOT = (52, 152, 219)
START = (39, 174, 96)
GOAL = (192, 57, 43)
VISITED = (189, 195, 199)
PANEL_BG = (30, 39, 46)
TEXT_COLOR = (236, 240, 241)
GRID_LINE = (210, 215, 220)


class HospitalVisualizer:
    """Render the hospital map, planned paths, and robot animation."""

    def __init__(self, hospital_map):
        """Initialize Pygame, create the window, and configure UI state."""
        pygame.init()
        pygame.font.init()

        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("Hospital Robot Navigation Simulator")
        self.clock = pygame.time.Clock()

        self.hospital_map = hospital_map
        self.robot = Robot(hospital_map, algorithm="astar")
        self.font = pygame.font.SysFont("Arial", 14)
        self.small_font = pygame.font.SysFont("Arial", 11)
        self.title_font = pygame.font.SysFont("Arial", 22, bold=True)

        self.current_algorithm = "astar"
        self.animation_speed = 0.05
        self.show_visited = False
        self.path_color = PATH_ASTAR
        self.running = True
        self.paused = False

        self.current_path = []
        self.path_progress = []
        self.visited_order = []
        self.visited_progress = []
        self.segment_details = []
        self.stats = self._empty_stats()
        self.current_goal_name = "Waiting"

    def draw_grid(self, screen):
        """Draw the hospital floor cells, grid lines, and room abbreviations."""
        labels = {
            "Pharmacy": "PH",
            "Room_101": "R1",
            "Room_102": "R2",
            "Room_103": "R3",
            "ICU": "ICU",
            "ER": "ER",
            "Laboratory": "LAB",
            "Reception": "REC",
        }

        for y in range(self.hospital_map.height):
            for x in range(self.hospital_map.width):
                cell_value = self.hospital_map.get_cell(x, y)
                cell_color = CORRIDOR
                if cell_value == 1:
                    cell_color = WALL
                elif cell_value == 2:
                    cell_color = HIGH_COST

                rect = pygame.Rect(x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                pygame.draw.rect(screen, cell_color, rect)
                pygame.draw.rect(screen, GRID_LINE, rect, 1)

        for name, (x, y) in locations.items():
            label_text = labels.get(name, name[:3].upper())
            text_surface = self.small_font.render(label_text, True, BLACK)
            text_rect = text_surface.get_rect(center=self._cell_center((x, y)))
            screen.blit(text_surface, text_rect)

    def draw_path(self, screen, path, color):
        """Highlight the active path cells and connect them with smooth lines."""
        if not path:
            return

        line_width = max(4, CELL_SIZE // 6)
        for index, position in enumerate(path):
            center = self._cell_center(position)
            highlight = pygame.Rect(0, 0, CELL_SIZE - 12, CELL_SIZE - 12)
            highlight.center = center
            pygame.draw.rect(screen, color, highlight, border_radius=6)

            if index > 0:
                previous_center = self._cell_center(path[index - 1])
                pygame.draw.line(screen, color, previous_center, center, line_width)

    def draw_robot(self, screen, position):
        """Draw the robot as a blue circle with an 'R' label."""
        center = self._cell_center(position)
        radius = CELL_SIZE // 3
        pygame.draw.circle(screen, ROBOT, center, radius)
        pygame.draw.circle(screen, WHITE, center, radius, 2)

        text_surface = self.font.render("R", True, WHITE)
        text_rect = text_surface.get_rect(center=center)
        screen.blit(text_surface, text_rect)

    def draw_visited_cells(self, screen, visited_cells):
        """Draw visited cells as soft gray overlays without connecting lines."""
        for position in visited_cells:
            overlay = pygame.Rect(0, 0, CELL_SIZE - 16, CELL_SIZE - 16)
            overlay.center = self._cell_center(position)
            pygame.draw.rect(screen, VISITED, overlay, border_radius=5)

    def draw_goals(self, screen):
        """Draw the start marker and the first three mission goals."""
        start_rect = pygame.Rect(0, 0, CELL_SIZE - 10, CELL_SIZE - 10)
        start_rect.center = self._cell_center(self.hospital_map.get_start())
        pygame.draw.rect(screen, START, start_rect, border_radius=5)
        start_text = self.font.render("ST", True, WHITE)
        screen.blit(start_text, start_text.get_rect(center=start_rect.center))

        for index, (_, goal_position) in enumerate(goals[:-1], start=1):
            goal_rect = pygame.Rect(0, 0, CELL_SIZE - 10, CELL_SIZE - 10)
            goal_rect.center = self._cell_center(goal_position)
            pygame.draw.rect(screen, GOAL, goal_rect, border_radius=5)
            goal_text = self.font.render(f"G{index}", True, WHITE)
            screen.blit(goal_text, goal_text.get_rect(center=goal_rect.center))

    def draw_panel(self, screen, stats):
        """Draw the right-side information panel and control legend."""
        panel_rect = pygame.Rect(20 * CELL_SIZE, 0, PANEL_WIDTH, WINDOW_HEIGHT)
        pygame.draw.rect(screen, PANEL_BG, panel_rect)

        x_origin = 20 * CELL_SIZE + 20
        y_position = 20

        def write(text, font, color=TEXT_COLOR):
            nonlocal y_position
            surface = font.render(text, True, color)
            screen.blit(surface, (x_origin, y_position))
            y_position += surface.get_height() + 6

        def separator():
            nonlocal y_position
            pygame.draw.line(
                screen,
                TEXT_COLOR,
                (x_origin, y_position),
                (WINDOW_WIDTH - 20, y_position),
                1,
            )
            y_position += 12

        write("HOSPITAL ROBOT", self.title_font)
        write("NAVIGATION SIM", self.title_font)
        separator()

        write(f"Algorithm: {self._algorithm_label()}", self.font)
        write(f"Status: {self.robot.status.replace('_', ' ')}", self.font)
        write(f"Goal: {self.current_goal_name}", self.font)
        write(f"Steps: {stats['steps']}", self.font)
        write(f"Visited: {stats['visited_nodes']}", self.font)
        write(f"Cost: {stats['path_cost']}", self.font)
        write(f"Time: {stats['execution_time']:.2f} ms", self.font)
        separator()

        write("CONTROLS:", self.font)
        write("[SPACE] Run/Pause", self.font)
        write("[B] Switch to BFS", self.font)
        write("[A] Switch to A*", self.font)
        write("[D] Switch to Dijkstra", self.font)
        write("[R] Reset", self.font)
        write("[V] Show Visited", self.font)
        write("[Q] Quit", self.font)
        separator()

        write("LEGEND:", self.font)
        self._draw_legend_item(screen, x_origin, y_position, CORRIDOR, "CORRIDOR")
        y_position += 26
        self._draw_legend_item(screen, x_origin, y_position, WALL, "WALL")
        y_position += 26
        self._draw_legend_item(screen, x_origin, y_position, HIGH_COST, "HIGH COST")
        y_position += 26
        self._draw_legend_item(screen, x_origin, y_position, self.path_color, "PATH")
        y_position += 26
        self._draw_legend_item(screen, x_origin, y_position, ROBOT, "ROBOT")
        y_position += 26
        self._draw_legend_item(screen, x_origin, y_position, GOAL, "GOAL")

    def animate_robot(self, screen, path, stats):
        """Animate the robot through the full mission while updating the panel."""
        if not path:
            return

        self.robot.current_position = path[0]
        self.robot.mission_log = []
        self.robot.total_steps = 0
        self.robot.total_cost = self.hospital_map.get_cost(*path[0])
        self.robot.status = "MOVING"

        total_steps = max(len(path) - 1, 0)
        last_index = len(path) - 1
        segment_cursor = 0

        for step_index, position in enumerate(path):
            if not self.running:
                return

            while self.paused and self.running:
                self._handle_animation_events()
                current_stats = self._build_live_stats(stats, step_index, position)
                self._draw_frame(screen, position, current_stats)
                pygame.display.flip()
                self.clock.tick(FPS)

            self._handle_animation_events()
            if not self.running:
                return

            self.robot.current_position = position
            self.robot.total_steps = step_index
            self.robot.total_cost = self._path_cost(path[: step_index + 1])
            self.path_progress = path[: step_index + 1]
            self.visited_progress = self._visited_prefix(step_index, total_steps)

            if segment_cursor < len(self.segment_details):
                segment = self.segment_details[segment_cursor]
                if step_index == segment["end_index"]:
                    self.robot.mission_log.append(segment["log_entry"])
                    self.robot.status = "ARRIVED"
                    self.current_goal_name = segment["goal_name"]
                    segment_cursor += 1
                else:
                    self.robot.status = "MOVING"

            if step_index == last_index:
                self.robot.status = "MISSION_COMPLETE"
                self.current_goal_name = "Mission Complete"

            current_stats = self._build_live_stats(stats, step_index, position)
            self._draw_frame(screen, position, current_stats)
            pygame.display.flip()

            if step_index < last_index:
                time.sleep(self.animation_speed)

        self.stats = dict(stats)

    def run(self):
        """Run the interactive Pygame event loop until the user quits."""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_q, pygame.K_ESCAPE):
                        self.running = False
                    elif event.key == pygame.K_SPACE:
                        if self.robot.status == "MOVING" and self.current_path:
                            self.paused = not self.paused
                        else:
                            self._run_current_mission()
                    elif event.key == pygame.K_b:
                        self.current_algorithm = "bfs"
                        self.robot.set_algorithm("bfs")
                        self.path_color = PATH_BFS
                        self._run_current_mission()
                    elif event.key == pygame.K_a:
                        self.current_algorithm = "astar"
                        self.robot.set_algorithm("astar")
                        self.path_color = PATH_ASTAR
                        self._run_current_mission()
                    elif event.key == pygame.K_d:
                        self.current_algorithm = "dijkstra"
                        self.robot.set_algorithm("dijkstra")
                        self.path_color = PATH_DIJKSTRA
                        self._run_current_mission()
                    elif event.key == pygame.K_r:
                        self._reset_visual_state()
                    elif event.key == pygame.K_v:
                        self.show_visited = not self.show_visited

            self._draw_frame(self.screen, self.robot.current_position, self.stats)
            pygame.display.flip()
            self.clock.tick(FPS)

        pygame.quit()

    def _draw_frame(self, screen, robot_position, stats):
        """Render the full scene for the current simulator state."""
        screen.fill(WHITE)
        self.draw_grid(screen)

        if self.show_visited and self.visited_progress:
            self.draw_visited_cells(screen, self.visited_progress)

        if self.path_progress:
            self.draw_path(screen, self.path_progress, self.path_color)

        self.draw_goals(screen)
        self.draw_robot(screen, robot_position)
        self.draw_panel(screen, stats)

    def _run_current_mission(self):
        """Plan the mission with the current algorithm and animate it."""
        self._reset_visual_state(keep_algorithm=True)
        self.robot.set_algorithm(self.current_algorithm)
        mission_bundle = self._prepare_mission_bundle()
        if mission_bundle is None:
            return

        self.current_path = mission_bundle["path"]
        self.segment_details = mission_bundle["segments"]
        self.visited_order = mission_bundle["visited_order"]
        self.stats = mission_bundle["stats"]
        if self.current_algorithm == "astar":
            self.path_color = PATH_ASTAR
        elif self.current_algorithm == "dijkstra":
            self.path_color = PATH_DIJKSTRA
        else:
            self.path_color = PATH_BFS
        self.current_goal_name = mission_bundle["segments"][0]["goal_name"] if mission_bundle["segments"] else "Waiting"
        self.animate_robot(self.screen, self.current_path, self.stats)

    def _prepare_mission_bundle(self):
        """Collect path, stats, segments, and visited order for the full mission."""
        if self.current_algorithm == "bfs":
            combined_path, total_stats = run_multi_goal_bfs(self.hospital_map)
            solver_class = BFS
        elif self.current_algorithm == "dijkstra":
            combined_path, total_stats = run_multi_goal_dijkstra(self.hospital_map)
            solver_class = Dijkstra
        else:
            combined_path, total_stats = run_multi_goal_astar(self.hospital_map)
            solver_class = AStar

        if not combined_path:
            return None

        segment_details = []
        visited_order = []
        merged_path = []
        current_start = self.hospital_map.get_start()

        for goal_name, goal_position in goals:
            solver = solver_class(self.hospital_map)
            segment_path = solver.search(current_start, goal_position)
            if not segment_path:
                return None

            segment_cost = self._segment_cost(segment_path, first_segment=not segment_details)
            segment_steps = max(len(segment_path) - 1, 0)
            visited_segment = self._collect_visited_sequence(current_start, goal_position)

            if merged_path:
                merged_path.extend(segment_path[1:])
            else:
                merged_path.extend(segment_path)

            segment_details.append(
                {
                    "goal_name": goal_name,
                    "goal_position": goal_position,
                    "end_index": len(merged_path) - 1,
                    "log_entry": {
                        "from": self._location_name(current_start),
                        "to": goal_name,
                        "steps": segment_steps,
                        "cost": segment_cost,
                        "algorithm": self._algorithm_label(),
                    },
                }
            )

            visited_order.extend(visited_segment)
            current_start = goal_position

        total_stats["steps"] = total_stats["path_length"]
        return {
            "path": combined_path,
            "segments": segment_details,
            "visited_order": visited_order,
            "stats": total_stats,
        }

    def _collect_visited_sequence(self, start, goal):
        """Recreate the visited order for visualization purposes."""
        if self.current_algorithm == "bfs":
            return self._collect_bfs_visited(start, goal)
        if self.current_algorithm == "dijkstra":
            return self._collect_dijkstra_visited(start, goal)
        return self._collect_astar_visited(start, goal)

    def _collect_bfs_visited(self, start, goal):
        """Collect BFS visited order using the same expansion rules as the solver."""
        queue = deque([start])
        visited = {start}
        visit_order = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        while queue:
            current = queue.popleft()
            visit_order.append(current)
            if current == goal:
                break

            current_x, current_y = current
            for dx, dy in directions:
                next_x = current_x + dx
                next_y = current_y + dy
                neighbor = (next_x, next_y)

                if not (0 <= next_x < self.hospital_map.width and 0 <= next_y < self.hospital_map.height):
                    continue
                if neighbor in visited:
                    continue
                if self.hospital_map.get_cell(next_x, next_y) == 1:
                    continue

                visited.add(neighbor)
                queue.append(neighbor)

        return visit_order

    def _collect_astar_visited(self, start, goal):
        """Collect A* visited order using the same priority rules as the solver."""
        open_set = []
        closed_set = set()
        g_score = {start: 0}
        visit_order = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        heapq.heappush(open_set, (self._heuristic(start, goal), 0, start))

        while open_set:
            _, current_g, current = heapq.heappop(open_set)
            if current in closed_set:
                continue

            closed_set.add(current)
            visit_order.append(current)
            if current == goal:
                break

            current_x, current_y = current
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
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        return visit_order

    def _collect_dijkstra_visited(self, start, goal):
        """Collect Dijkstra visited order using traveled cost priorities."""
        open_set = []
        closed_set = set()
        g_score = {start: 0}
        visit_order = []
        directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

        heapq.heappush(open_set, (0, start))

        while open_set:
            current_cost, current = heapq.heappop(open_set)
            if current in closed_set:
                continue

            closed_set.add(current)
            visit_order.append(current)
            if current == goal:
                break

            current_x, current_y = current
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
                tentative_g = current_cost + movement_cost
                if tentative_g < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_g
                    heapq.heappush(open_set, (tentative_g, neighbor))

        return visit_order

    def _build_live_stats(self, planned_stats, step_index, position):
        """Create panel stats for the current animation frame."""
        live_stats = dict(planned_stats)
        live_stats["steps"] = step_index
        live_stats["path_cost"] = self._path_cost(self.path_progress) if self.path_progress else self.hospital_map.get_cost(*position)
        if self.show_visited:
            live_stats["visited_nodes"] = len(self.visited_progress)
        return live_stats

    def _visited_prefix(self, step_index, total_steps):
        """Reveal visited cells progressively as the robot advances."""
        if not self.show_visited or not self.visited_order:
            return []

        if total_steps <= 0:
            return list(dict.fromkeys(self.visited_order[:1]))

        reveal_ratio = step_index / total_steps
        reveal_count = max(1, int(reveal_ratio * len(self.visited_order)))
        return list(dict.fromkeys(self.visited_order[:reveal_count]))

    def _draw_legend_item(self, screen, x_origin, y_origin, color, label):
        """Draw one colored legend item in the side panel."""
        marker = pygame.Rect(x_origin, y_origin + 2, 18, 18)
        pygame.draw.rect(screen, color, marker, border_radius=3)
        text_surface = self.font.render(label, True, TEXT_COLOR)
        screen.blit(text_surface, (x_origin + 28, y_origin))

    def _segment_cost(self, path, first_segment=False):
        """Calculate segment cost while counting the start cell only once."""
        if first_segment:
            return sum(self.hospital_map.get_cost(x, y) for x, y in path)
        return sum(self.hospital_map.get_cost(x, y) for x, y in path[1:])

    def _path_cost(self, path):
        """Calculate the total cost for the traversed path prefix."""
        return sum(self.hospital_map.get_cost(x, y) for x, y in path)

    def _cell_center(self, position):
        """Convert grid coordinates to pixel-center coordinates."""
        x, y = position
        return (x * CELL_SIZE + CELL_SIZE // 2, y * CELL_SIZE + CELL_SIZE // 2)

    def _heuristic(self, point_a, point_b):
        """Return Manhattan distance between two grid cells."""
        return abs(point_a[0] - point_b[0]) + abs(point_a[1] - point_b[1])

    def _location_name(self, position):
        """Resolve a named hospital location from coordinates."""
        for name, coordinates in locations.items():
            if coordinates == position:
                return name
        return "Unknown"

    def _algorithm_label(self):
        """Return a friendly label for the current pathfinding algorithm."""
        if self.current_algorithm == "astar":
            return "A*"
        if self.current_algorithm == "dijkstra":
            return "Dijkstra"
        return "BFS"

    def _empty_stats(self):
        """Return a default stats dictionary for the side panel."""
        return {
            "algorithm": self._algorithm_label() if hasattr(self, "current_algorithm") else "A*",
            "path_length": 0,
            "steps": 0,
            "visited_nodes": 0,
            "execution_time": 0.0,
            "path_cost": 0,
        }

    def _reset_visual_state(self, keep_algorithm=False):
        """Reset the visualizer to its initial idle state."""
        if not keep_algorithm:
            self.current_algorithm = "astar"
            self.path_color = PATH_ASTAR
            self.robot.set_algorithm("astar")

        self.paused = False
        self.current_path = []
        self.path_progress = []
        self.visited_order = []
        self.visited_progress = []
        self.segment_details = []
        self.stats = self._empty_stats()
        self.current_goal_name = "Waiting"
        self.robot.current_position = self.hospital_map.get_start()
        self.robot.mission_log = []
        self.robot.total_steps = 0
        self.robot.total_cost = 0
        self.robot.status = "IDLE"

    def _handle_animation_events(self):
        """Handle quit and pause events while the robot is animating."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_d:
                    self.current_algorithm = "dijkstra"
                    self.path_color = PATH_DIJKSTRA
                elif event.key == pygame.K_v:
                    self.show_visited = not self.show_visited


if __name__ == "__main__":
    hospital = HospitalMap()
    viz = HospitalVisualizer(hospital)
    viz.run()
