import numpy as np
import server_readings as gr
import math
import obstacles


def grassfire(grid, start, goal):
    print("The algorithm starts...")
    global distance
    distance = np.full(grid.shape, 0)
#    distance[start] = 0

    queue = [start]  # store the cells that have not been processed
    while queue:  # while the queue is not empty
        cell = queue.pop(0)
        neighbors = get_neighbors(grid, cell)
        for neighbor in neighbors:
            if distance[(neighbor[0], neighbor[1])] > distance[cell] + 1:
                distance[(neighbor[0], neighbor[1])] = distance[cell] + 1
                queue.append(neighbor)

    path = []
    cell = goal
    while cell != start:
        path.append(cell)
        cell = get_parent(grid, cell)

    path.reverse()
    return path


# A function to get the neighbours of the cell
def get_neighbors(grid, cell):
    print("Get neighbors...")
    # get the coordinates of the current cell
    x, y = cell
    # get the neighbors of the current cell
    neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    # remove any neighbors that are in obstacles grid or out of the grid dimensions
    neighbors = [neighbor for neighbor in neighbors
                 if start[0] <= neighbor[0] < goal[0] and
                 start[1] <= neighbor[1] < goal[1]]
    return neighbors


# A function to get the parent of the node of the graph
def get_parent(grid, cell):
    print("Get parent...")
    x, y = cell
    if cell == start:
        return None
    neighbors = get_neighbors(grid, cell)  # get the neighbors of the current node
    # find the neighbor of the smallest distance
    min_distance = np.inf
    min_neighbor = None
    for neighbor in neighbors:
        if distance[(neighbor[0], neighbor[1])] < min_distance:
            min_distance = distance[(neighbor[0], neighbor[1])]
            min_neighbor = neighbor

    return min_neighbor


start = (gr.start_x, gr.start_y)
goal = (gr.end_x, gr.end_y)
# height = (gr.end_y - gr.start_y)
height = math.ceil(goal[1]) - math.floor(start[1])
# width = (gr.end_x - gr.start_x)
width = math.ceil(goal[0]) - math.floor(start[0])
grid = np.zeros((height, width))

path = grassfire(grid, start, goal)
print("path = ", path)