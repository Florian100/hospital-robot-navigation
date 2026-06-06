"""Entry point for the hospital robot navigation simulation project."""

import sys
import os

os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"


# Track import issues so menu actions can fail gracefully instead of crashing.
IMPORT_ERRORS = {}

try:
    from hospital_map import HospitalMap, locations, goals
except ImportError as error:
    HospitalMap = None
    locations = {}
    goals = []
    IMPORT_ERRORS["hospital_map"] = error

sys.path.append("./algorithms")
try:
    from bfs import BFS, run_multi_goal_bfs
except ImportError as error:
    BFS = None
    run_multi_goal_bfs = None
    IMPORT_ERRORS["bfs"] = error

try:
    from astar import AStar, run_multi_goal_astar
except ImportError as error:
    AStar = None
    run_multi_goal_astar = None
    IMPORT_ERRORS["astar"] = error

try:
    from dijkstra import Dijkstra, run_multi_goal_dijkstra
except ImportError as error:
    Dijkstra = None
    run_multi_goal_dijkstra = None
    IMPORT_ERRORS["dijkstra"] = error

try:
    from robot import Robot
except ImportError as error:
    Robot = None
    IMPORT_ERRORS["robot"] = error

sys.path.append("./utils")
try:
    from comparator import AlgorithmComparator
except ImportError as error:
    AlgorithmComparator = None
    IMPORT_ERRORS["comparator"] = error

try:
    from export_results import export_all
except ImportError as error:
    export_all = None
    IMPORT_ERRORS["export_results"] = error

sys.path.append("./visualization")
try:
    from display import HospitalVisualizer
except ImportError as error:
    HospitalVisualizer = None
    IMPORT_ERRORS["display"] = error


def print_menu():
    """Print the main menu for the simulator."""
    menu_lines = [
        "+----------------------------------------------+",
        "|   HOSPITAL ROBOT NAVIGATION SIMULATOR        |",
        "|   Elemente te Robotikes dhe Automatizimit    |",
        "+----------------------------------------------+",
        "|  1. Run Terminal Simulation (A*)             |",
        "|  2. Run Terminal Simulation (BFS)            |",
        "|  3. Run Terminal Simulation (Dijkstra)       |",
        "|  4. Compare All Algorithms (+ charts)        |",
        "|  5. Launch Visual Simulator (Pygame)         |",
        "|  6. Export Results to CSV                    |",
        "|  7. Show Hospital Map                        |",
        "|  0. Exit                                     |",
        "+----------------------------------------------+",
    ]

    for line in menu_lines:
        print(line)


def run_terminal_simulation(algorithm):
    """Create a robot and execute the full mission in the terminal."""
    if HospitalMap is None or Robot is None:
        _print_import_error("robot")
        return

    hospital = HospitalMap()
    robot = Robot(hospital, algorithm=algorithm)
    robot.execute_mission()
    print(robot.get_mission_report())


def run_comparison():
    """Run the BFS, Dijkstra, and A* comparison and generate charts."""
    if HospitalMap is None or AlgorithmComparator is None:
        _print_import_error("comparator")
        return

    hospital = HospitalMap()
    comparator = AlgorithmComparator(hospital)
    single_results = comparator.run_single_goal_comparison()
    multi_results = comparator.run_multi_goal_comparison()

    comparator.print_comparison_table(single_results, multi_results)
    comparator.generate_comparison_charts(single_results, multi_results)
    print("\nChart saved as 'comparison_results.png'")


def export_results_to_csv():
    """Export single-goal and multi-goal results to the results folder."""
    if HospitalMap is None or export_all is None:
        _print_import_error("export_results")
        return

    hospital = HospitalMap()
    export_all(hospital)
    print("\nResults exported to results/ folder!")


def launch_visual_simulator():
    """Open the Pygame visual simulator."""
    if HospitalMap is None or HospitalVisualizer is None:
        _print_import_error("display")
        return

    hospital = HospitalMap()
    viz = HospitalVisualizer(hospital)
    viz.run()


def show_map():
    """Print the symbolic hospital map and the named locations."""
    if HospitalMap is None:
        _print_import_error("hospital_map")
        return

    hospital = HospitalMap()
    hospital.print_map()
    print("\nLocations:")
    print(locations)
    print("\nGoals:")
    print(goals)


def _print_import_error(module_name):
    """Display a friendly import error message for missing modules."""
    error = IMPORT_ERRORS.get(module_name)
    if error is None:
        print("The requested feature is currently unavailable.")
        return

    print(f"Unable to load required module: {module_name}")
    print(f"Reason: {error}")


if __name__ == "__main__":
    print("\nWelcome to Hospital Robot Navigation Simulator!")
    print("Project: Elemente te Robotikes dhe Automatizimit\n")

    if os.name == "nt":
        os.system("")

    hospital = HospitalMap() if HospitalMap is not None else None
    if hospital is None and "hospital_map" in IMPORT_ERRORS:
        _print_import_error("hospital_map")

    while True:
        print_menu()
        choice = input("Select option (0-7): ").strip()

        if choice == "1":
            run_terminal_simulation("astar")
        elif choice == "2":
            run_terminal_simulation("bfs")
        elif choice == "3":
            run_terminal_simulation("dijkstra")
        elif choice == "4":
            run_comparison()
        elif choice == "5":
            launch_visual_simulator()
        elif choice == "6":
            export_results_to_csv()
        elif choice == "7":
            show_map()
        elif choice == "0":
            print("\nGoodbye! Mission Complete!")
            sys.exit(0)
        else:
            print("Invalid option. Please try again.")

        input("\nPress ENTER to continue...")
