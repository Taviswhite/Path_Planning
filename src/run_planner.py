from graph import GridGraph, Cell
from graph_search import a_star_search
from utils import generate_plan_file

# Load the map
graph = GridGraph(file_path="/Users/Tavis/Downloads/Path_Planning/data/maze2.map")

# Get cell coordinates
print("Enter START cell (i, j):")
start_i = int(input("  i: "))
start_j = int(input("  j: "))

print("Enter GOAL cell (i, j):")
goal_i = int(input("  i: "))
goal_j = int(input("  j: "))

start = Cell(start_i, start_j)
goal = Cell(goal_i, goal_j)

# Alternative: Use position-based input instead
# start = graph.pos_to_cell(0.0, 0.0)
# goal = graph.pos_to_cell(1.0, 1.0)

# Run search and generate file
path = a_star_search(graph, start, goal)

if path:
    generate_plan_file(graph, start, goal, path, algo="A*", out_name="output.planner")
    print(f"Path found: {len(path)} cells")
else:
    print("No path found")
