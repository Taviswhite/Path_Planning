import numpy as np
from .graph import Cell
from .utils import trace_path

"""
General graph search instructions:

First, define the correct data type to keep track of your visited cells
and add the start cell to it. If you need to initialize any properties
of the start cell, do that too.

Next, implement the graph search function. When you find a path, use the
trace_path() function to return a path given the goal cell and the graph. You
must have kept track of the parent of each node correctly and have implemented
the graph.get_parent() function for this to work. If you do not find a path,
return an empty list.

To visualize which cells are visited in the navigation webapp, save each
visited cell in the list in the graph class as follows:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j are the cell indices of the visited cell you want to
visualize.
"""


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    stack = [start]
    visited = set()
    visited.add((start.i, start.j))

    graph.visited_cells.append(Cell(start.i, start.j))

    while stack:
        current = stack.pop()

        if current == goal:
            return trace_path(goal, graph)

        for nbr in graph.get_neighbors(current):
            key = (nbr.i, nbr.j)
            if key not in visited:
                visited.add(key)
                graph.set_parent(nbr, current)
                graph.visited_cells.append(Cell(nbr.i, nbr.j))
                stack.append(nbr)


    """TODO (P3): Implement DFS (optional)."""

    # If no path was found, return an empty list.
    return []


def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.
        
    queue = [start]
    visited = set()
    visited.add((start.i, start.j))

    graph.visited_cells.append(Cell(start.i, start.j))

    while queue:
        current = queue.pop(0)

        if current == goal:
            return trace_path(goal, graph)

        for nbr in graph.get_neighbors(current):
            key = (nbr.i, nbr.j)
            if key not in visited:
                visited.add(key)
                graph.set_parent(nbr, current)
                graph.visited_cells.append(Cell(nbr.i, nbr.j))
                queue.append(nbr)

    """TODO (P3): Implement BFS."""

    # If no path was found, return an empty list.
    return []


def a_star_search(graph, start, goal):
    """A* Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.
   
    # f = g + h
    # priority queue stores (f, g, cell)
    open_list = []
    heapq.heappush(open_list, (0, 0, start))

    graph.set_cost(start, 0)
    visited = set()
    visited.add((start.i, start.j))

    graph.visited_cells.append(Cell(start.i, start.j))

    while open_list:
        f, g, current = heapq.heappop(open_list)

        if current == goal:
            return trace_path(goal, graph)

        for nbr in graph.get_neighbors(current):

            tentative_g = graph.get_cost(current) + 1
            old_cost = graph.get_cost(nbr)

            if old_cost is None or tentative_g < old_cost:
                graph.set_cost(nbr, tentative_g)
                h = graph.heuristic(nbr, goal)
                f_new = tentative_g + h

                graph.set_parent(nbr, current)

                heapq.heappush(open_list, (f_new, tentative_g, nbr))

                key = (nbr.i, nbr.j)
                if key not in visited:
                    visited.add(key)
                    graph.visited_cells.append(Cell(nbr.i, nbr.j))


    """TODO (P3): Implement A*."""

    # If no path was found, return an empty list.
    return []
