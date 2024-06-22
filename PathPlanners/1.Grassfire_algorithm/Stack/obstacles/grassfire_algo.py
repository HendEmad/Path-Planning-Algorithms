import server_readings as gr
import math
import obstacles as ob
import matplotlib.pyplot as plt
import numpy as np


class Graph:
    def __init__(self, start_x, start_y, end_x, end_y, obstacles):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.width = int(end_x) - int(start_x) + 1
        self.height = int(end_y) - int(start_y) + 1
        self.graph = [[{'distance': float('inf')} for j in range(self.height)] for i in range(self.width)]
        self.obstacles = obstacles
        self.graph[int(start_x) - int(start_x)][int(start_y) - int(start_y)]['distance'] = 0

    # A function to check if the node is inside any of the obstacle regions
    def is_obstacle(self, node):
        for obstacle in self.obstacles:
            if len(obstacle) == 3:  # Circle obstacle
                x, y, r = obstacle
                if math.sqrt((node[0] - x) ** 2 + (node[1] - y) ** 2) <= r:
                    return True
            elif len(obstacle) == 4:  # Rectangle obstacle
                x1, y1, x2, y2 = obstacle
                if x1 <= node[0] <= x2 and y1 <= node[1] <= y2:
                    return True
            elif len(obstacle) == 6:  # Triangle obstacle
                x1, y1, x2, y2, x3, y3 = obstacle
                a = (x1 - node[0]) * (y2 - y1) - (x2 - x1) * (y1 - node[1])
                b = (x2 - node[0]) * (y3 - y2) - (x3 - x2) * (y2 - node[1])
                c = (x3 - node[0]) * (y1 - y3) - (x1 - x3) * (y3 - node[1])
                if (a >= 0 and b >= 0 and c >= 0) or (a <= 0 and b <= 0 and c <= 0):
                    return True
        return False

    # A function returns the neighboring nodes of a given node that are not obstacles
    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        return [n for n in neighbors
                if 0 <= n[0] < self.width and 0 <= n[1] < self.height and not self.is_obstacle(n)]

    # A function returns the distance value of a given node
    def get_distance(self, node):
        return self.graph[node[0]][node[1]]['distance']

    # A function to set the distance value of a given node
    def set_distance(self, node, distance):
        self.graph[node[0]][node[1]]['distance'] = distance

    # A function to run the grassfire algorithm
    def grassfire(self):
        queue = [(int(self.start_x) - int(self.start_x), int(self.start_y) - int(self.start_y))]

        while queue:
            current = queue.pop()  # stack
            current_distance = self.get_distance(current)
            neighbors = self.get_neighbors(current)

            for n in neighbors:
                neighbor_distance = self.get_distance(n)
                if neighbor_distance == float('inf'):
                    self.set_distance(n, current_distance + 1)
                    queue.append(n)

        path = [(int(self.end_x) - int(self.start_x), int(self.end_y) - int(self.start_y))]
        current_distance = self.get_distance(path[0])
        current_node = path[0]

        while current_distance > 0:
            neighbors = self.get_neighbors(current_node)

            for n in neighbors:
                neighbor_distance = self.get_distance(n)
                if neighbor_distance < current_distance:
                    current_distance = neighbor_distance
                    current_node = n

            path.append(current_node)

        return [(x + float(self.start_x), y + float(self.start_y)) for x, y in path[::-1]]


# A decorator function [function as input and new function as output]
def calculate_execution_time(func):
    """
        import time

        def wrapper(*args):
        # for average path planning time metric
            wrapper.total_time += time.time() + wrapper.last_call_time
            wrapper.last_call_time = time.time()
            result = func(*args)
            return result
        wrapper.total_time = 0.0
        wrapper.last_call_time = time.time()
        """

    """
    # for relative standard deviation metric
    import time

    def wrapper(*args):
        start_time = time.time()
        result = func(*args)
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time
    """

    import time

    def wrapper(*args):
        start_time = time.time()
        result = func(*args)
        end_time = time.time()
        execution_time = end_time - start_time
        return result

    return wrapper


# A function to use the Graph class to find the shortest distance using grassfire algorithm
@calculate_execution_time
def run_grassfire(start_x, start_y, end_x, end_y, obstacles):
    graph = Graph(start_x, start_y, end_x, end_y, obstacles)
    return graph.grassfire()


obstacles = ob.create_obstacles()
print(obstacles)

path = run_grassfire(gr.start_x, gr.start_y, gr.end_x, gr.end_y, obstacles)
print(f"Path: {path}")

width = float(gr.end_x) - float(gr.start_x) + 1
height = height = float(gr.end_y) - float(gr.start_y) + 1
print("This path performs on area of ", width, "m x ", height, "m")
