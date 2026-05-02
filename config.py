"""Global constants and default parameters for the trajectory planner."""

# ---------------------------------------------------------------------------
# Grid / world dimensions
# ---------------------------------------------------------------------------
GRID_ROWS = 50
GRID_COLS = 50
CELL_SIZE = 1  # Each cell is 1×1 in continuous space

# ---------------------------------------------------------------------------
# Default start / goal positions  (row, col)
# ---------------------------------------------------------------------------
DEFAULT_START = (2, 2)
DEFAULT_GOAL = (47, 47)

# ---------------------------------------------------------------------------
# A* defaults
# ---------------------------------------------------------------------------
ASTAR_HEURISTICS = ("Manhattan", "Euclidean", "Diagonal")
DEFAULT_HEURISTIC = "Euclidean"

# ---------------------------------------------------------------------------
# RRT defaults
# ---------------------------------------------------------------------------
RRT_STEP_SIZE = 1.5
RRT_MAX_ITER = 5000
RRT_GOAL_BIAS = 0.1
RRT_GOAL_THRESHOLD = 1.5

# ---------------------------------------------------------------------------
# Dynamic obstacle defaults
# ---------------------------------------------------------------------------
DYNAMIC_LINEAR_SPEED = 0.3       # cells per tick
DYNAMIC_CIRCULAR_SPEED = 0.05    # radians per tick
DYNAMIC_CIRCULAR_RADIUS = 5.0    # orbit radius

# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------
ANIMATION_INTERVAL_MS = 30       # milliseconds between frames
STEPS_PER_FRAME = 1              # algorithm iterations per animation tick

# ---------------------------------------------------------------------------
# Colours  (Matplotlib compatible)
# ---------------------------------------------------------------------------
COLOR_FREE = "#FFFFFF"
COLOR_OBSTACLE = "#2C3E50"
COLOR_DYNAMIC_OBSTACLE = "#8E44AD"
COLOR_START = "#E74C3C"
COLOR_GOAL = "#F1C40F"
COLOR_OPEN_SET = "#85C1E9"
COLOR_CLOSED_SET = "#D5DBDB"
COLOR_PATH = "#27AE60"
COLOR_RRT_TREE = "#3498DB"
COLOR_RRT_NODE = "#2980B9"
COLOR_GRID_LINE = "#ECF0F1"
