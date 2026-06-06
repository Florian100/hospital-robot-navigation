"""2D hospital map model for a robot navigation simulation."""

from math import inf


# Define the fixed coordinates for all named hospital locations.
locations = {
    "Pharmacy": (1, 1),
    "Room_101": (14, 4),
    "Room_102": (14, 8),
    "Room_103": (14, 12),
    "ICU": (10, 15),
    "ER": (3, 15),
    "Laboratory": (9, 9),
    "Reception": (17, 18),
}


# Define the ordered mission sequence for the robot.
goals = [
    ("Room_101", locations["Room_101"]),
    ("Laboratory", locations["Laboratory"]),
    ("ICU", locations["ICU"]),
    ("Pharmacy", locations["Pharmacy"]),
]


class HospitalMap:
    """Represent a 20x20 hospital floor with walls, corridors, and priority zones."""

    def __init__(self):
        """Initialize the map grid, structural walls, and named hospital zones."""
        self.width = 20
        self.height = 20
        self.grid = [[0 for _ in range(self.width)] for _ in range(self.height)]
        self.locations = dict(locations)
        self.goals = list(goals)
        self.start = self.locations["Pharmacy"]
        self.room_lookup = {coords: name for name, coords in self.locations.items()}

        # Mark the outer border as walls to contain the hospital floor plan.
        for x in range(self.width):
            self.grid[0][x] = 1
            self.grid[self.height - 1][x] = 1
        for y in range(self.height):
            self.grid[y][0] = 1
            self.grid[y][self.width - 1] = 1

        # Add internal walls to create corridors and separate hospital sections.
        wall_segments = [
            ((4, 1), (4, 13)),
            ((6, 6), (6, 18)),
            ((12, 1), (12, 17)),
            ((16, 3), (16, 18)),
            ((1, 6), (4, 6)),
            ((4, 13), (12, 13)),
            ((6, 3), (11, 3)),
            ((6, 17), (16, 17)),
        ]

        for start, end in wall_segments:
            self._fill_line(start, end, 1)

        # Cut openings in selected walls so the map stays navigable.
        doorways = [
            (4, 3),
            (4, 9),
            (6, 10),
            (8, 3),
            (10, 13),
            (12, 5),
            (12, 9),
            (12, 15),
            (14, 17),
            (16, 11),
        ]

        for x, y in doorways:
            self.grid[y][x] = 0

        # Mark the ICU and ER zones as high-traffic cells with higher movement cost.
        high_traffic_cells = [
            (2, 14),
            (3, 14),
            (2, 15),
            (3, 15),
            (4, 15),
            (9, 14),
            (10, 14),
            (11, 14),
            (9, 15),
            (10, 15),
            (11, 15),
            (10, 16),
        ]

        for x, y in high_traffic_cells:
            self.grid[y][x] = 2

    def _fill_line(self, start, end, value):
        """Fill a horizontal or vertical line segment with a given cell value."""
        start_x, start_y = start
        end_x, end_y = end

        if start_x == end_x:
            step = 1 if end_y >= start_y else -1
            for y in range(start_y, end_y + step, step):
                self.grid[y][start_x] = value
            return

        if start_y == end_y:
            step = 1 if end_x >= start_x else -1
            for x in range(start_x, end_x + step, step):
                self.grid[start_y][x] = value
            return

        raise ValueError("Only horizontal or vertical lines are supported.")

    def get_cell(self, x, y):
        """Return the raw value stored in the requested cell."""
        return self.grid[y][x]

    def get_cost(self, x, y):
        """Return the movement cost for a cell based on its type."""
        cell_value = self.get_cell(x, y)
        if cell_value == 1:
            return inf
        if cell_value == 2:
            return 5
        return 1

    def get_room_name(self, x, y):
        """Return the room or zone name for an exact named coordinate."""
        return self.room_lookup.get((x, y))

    def get_start(self):
        """Return the robot start position at the Pharmacy."""
        return self.start

    def get_goals(self):
        """Return only the ordered goal coordinates for the mission."""
        return [coords for _, coords in self.goals]

    def print_map(self):
        """Print the hospital map using compact symbols for each cell type."""
        goal_markers = {
            self.goals[0][1]: "G1",
            self.goals[1][1]: "G2",
            self.goals[2][1]: "G3",
        }

        for y in range(self.height):
            row_symbols = []
            for x in range(self.width):
                position = (x, y)

                if position == self.start:
                    row_symbols.append("ST")
                elif position in goal_markers:
                    row_symbols.append(goal_markers[position])
                else:
                    cell_value = self.grid[y][x]
                    if cell_value == 1:
                        row_symbols.append("##")
                    elif cell_value == 2:
                        row_symbols.append("!!")
                    else:
                        row_symbols.append("  ")
            print("".join(row_symbols))


if __name__ == "__main__":
    # Create the hospital map and print the symbolic layout.
    hospital = HospitalMap()
    hospital.print_map()

    # Print key navigation data for quick inspection.
    print("Start:", hospital.get_start())
    print("Goals:", hospital.get_goals())
    print("Cost at ER:", hospital.get_cost(*locations["ER"]))
