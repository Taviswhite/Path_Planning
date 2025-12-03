import numpy as np
import heapq
from collections import deque
from graph import Cell
from utils import trace_path

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
    
    # Set start parent to None
    graph.set_parent(start, None)

    while stack:
        current = stack.pop()
        
        key = (current.i, current.j)
        if key in visited:
            continue
            
        visited.add(key)
        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)

        for nbr in graph.find_neighbors(current.i, current.j):
            nbr_key = (nbr.i, nbr.j)
            if nbr_key not in visited and not graph.check_collision(nbr.i, nbr.j):
                # Only set parent if not already set
                if graph.parent_i[nbr.j, nbr.i] == -1 and graph.parent_j[nbr.j, nbr.i] == -1:
                    graph.set_parent(nbr, current)
                    stack.append(nbr)

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
    
    queue = deque([start])  # Use deque for O(1) operations
    visited = set()
    visited.add((start.i, start.j))
    
    # Set start parent to None
    graph.set_parent(start, None)

    while queue:
        current = queue.popleft()
        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)

        for nbr in graph.find_neighbors(current.i, current.j):
            key = (nbr.i, nbr.j)
            if key not in visited and not graph.check_collision(nbr.i, nbr.j):
                visited.add(key)
                graph.set_parent(nbr, current)
                queue.append(nbr)

    # If no path was found, return an empty list.
    return []


def a_star_search(graph, start, goal):
    """A* Search algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.
    
    def heuristic(a, b):
        """Manhattan distance heuristic."""
        return abs(a.i - b.i) + abs(a.j - b.j)
    
    # Priority queue stores (f_score, cell)
    open_list = []
    heapq.heappush(open_list, (0, start))
    
    # Initialize start node
    graph.dist[start.j, start.i] = 0
    graph.set_parent(start, None)
    
    closed_set = set()

    while open_list:
        _, current = heapq.heappop(open_list)
        
        key = (current.i, current.j)
        if key in closed_set:
            continue
            
        closed_set.add(key)
        graph.visited_cells.append(Cell(current.i, current.j))

        if current.i == goal.i and current.j == goal.j:
            return trace_path(goal, graph)

        for nbr in graph.find_neighbors(current.i, current.j):
            if graph.check_collision(nbr.i, nbr.j):
                continue
                
            nbr_key = (nbr.i, nbr.j)
            if nbr_key in closed_set:
                continue

            tentative_g = graph.dist[current.j, current.i] + 1
            
            if tentative_g < graph.dist[nbr.j, nbr.i]:
                graph.dist[nbr.j, nbr.i] = tentative_g
                h = heuristic(nbr, goal)
                f_score = tentative_g + h
                
                graph.set_parent(nbr, current)
                heapq.heappush(open_list, (f_score, nbr))

    # If no path was found, return an empty list.
    return []
