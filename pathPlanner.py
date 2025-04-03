# Import any libraries required

# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION

## The main A* search function for path planning
# This function follows the standard A* formulation and returns an ordered list of (col, row) coordinates.
# The algorithm utilizes heuristic-based best-first search combined with uniform-cost search to find an optimal path.
# It maintains an OPEN set of nodes to be evaluated and a CLOSED set of nodes already explored.

def do_a_star(grid, start, end, display_message):
    # Define grid dimensions (columns, rows)
    COL = len(grid)   # Number of columns
    ROW = len(grid[0])  # Number of rows

    # Heuristic function: Euclidean distance as an admissible and consistent heuristic
    # The heuristic function h(n) estimates the cost from node n to the goal node.
    def heuristic(cell, goal):
        cx, cy = cell
        gx, gy = goal
        return ((cx - gx) ** 2 + (cy - gy) ** 2) ** 0.5  # Euclidean distance (consistent heuristic)

    # If the start and goal nodes are identical, return a trivial solution immediately
    if start == end:
        display_message("Start and goal are the same, returning trivial path")
        return [start]

    # Initialize OPEN and CLOSED sets for A* search
    open_set = set([start])  # OPEN set: nodes pending evaluation
    closed_set = set()  # CLOSED set: nodes already evaluated

    # Cost dictionaries:
    # g_cost(n) - Actual cost from start node to node n (computed path cost)
    # f_cost(n) = g_cost(n) + h_cost(n) (total estimated cost to goal)
    g_cost = {start: 0}  # Initialize g(n) with 0 for the start node
    f_cost = {start: heuristic(start, end)}  # Initialize f(n) with heuristic estimate

    # Dictionary to store the best path traversal information (backtracking mechanism)
    came_from = {}  # Stores parent nodes for path reconstruction

    # Begin the A* search loop
    display_message(f"Starting A* search from {start} to {end}")
    while open_set:
        # Select the node in OPEN set with the lowest f_cost (best candidate for expansion)
        current = min(open_set, key=lambda node: f_cost.get(node, float('inf')))

        # If the goal node is reached, reconstruct and return the optimal path
        if current == end:
            display_message("Goal reached! Reconstructing optimal path.")
            path = []
            node = current
            while node in came_from:  # Backtrack to retrieve path from start to goal
                path.append(node)
                node = came_from[node]
            path.append(start)  # Ensure start node is included
            path.reverse()  # Reverse the path to get correct order from start -> goal
            display_message(f"Optimal path found: {path}")
            return path

        # Move current node from OPEN to CLOSED (mark as explored)
        open_set.remove(current)
        closed_set.add(current)

        # Explore four adjacent neighbors (4-connected grid: up, down, left, right)
        cx, cy = current
        neighbors = [(cx+1, cy), (cx-1, cy), (cx, cy+1), (cx, cy-1)]
        for neighbor in neighbors:
            nx, ny = neighbor

            # Skip neighbors that are out of bounds
            if nx < 0 or nx >= COL or ny < 0 or ny >= ROW:
                continue

            # Skip neighbors that are obstacles (grid value 0 represents impassable terrain)
            if grid[nx][ny] == 0:
                continue

            # Skip neighbors that are already evaluated (in CLOSED set)
            if neighbor in closed_set:
                continue

            # Compute tentative g(n) (cost from start to neighbor through current node)
            tentative_g = g_cost[current] + 1  # Assumes uniform cost for movement

            # If this neighbor is not yet in OPEN set, or we found a lower-cost path
            if neighbor not in open_set or tentative_g < g_cost.get(neighbor, float('inf')):
                came_from[neighbor] = current  # Store parent for backtracking
                g_cost[neighbor] = tentative_g  # Update g(n)
                f_cost[neighbor] = tentative_g + heuristic(neighbor, end)  # Compute f(n)

                # If the neighbor is newly discovered, add to OPEN set for evaluation
                if neighbor not in open_set:
                    open_set.add(neighbor)
    
    # If the algorithm terminates without reaching the goal, return an empty path (no solution found)
    display_message("No path found - returning empty list", "WARN")
    return []

# end of file