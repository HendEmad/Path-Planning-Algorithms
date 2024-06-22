import heapq
import math
from collections import deque
import server_readings as gr
import random
import numpy as np
from obstacles import create_obstacles


class Graph:
    def __init__(self, start_x, start_y, end_x, end_y, weights, obstacles):
        self.start_x = int(start_x)
        self.start_y = int(start_y)
        self.end_x = int(end_x)
        self.end_y = int(end_y)
        self.width = self.end_x - self.start_x + 1
        self.height = self.end_y - self.start_y + 1
        self.weights = {}  # weights[node1][node] = the weight between node 1 and node 2
        self.graph = np.empty((self.width, self.height), dtype=object)
        self.obstacles = obstacles

        for x in range(self.start_x, self.end_x + 1):
            for y in range(self.start_y, self.end_y + 1):
                self.graph[x - self.start_x, y - self.start_y] = {'distance': math.inf,
                                                                  'parent': None,
                                                                  'neighbors': {}}

        self.graph[0, 0]['distance'] = 0
        for x in range(self.start_x, self.end_x + 1):
            for y in range(self.start_y, self.end_y + 1):
                self.set_weights((x, y), weights)

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        return [(nx, ny) for nx, ny in neighbors if
                0 <= nx - self.start_x < self.width and
                0 <= ny - self.start_y < self.height and not
                self.is_obstacle((nx, ny))]

    def get_distance(self, node):
        x, y = node
        return self.graph[x - self.start_x, y - self.start_y]['distance']

    def set_distance(self, node, distance):
        x, y = node
        self.graph[x - self.start_x, y - self.start_y]['distance'] = distance

    def set_weights(self, node, weights):
        neighbors = self.get_neighbors(node)
        self.weights[node] = {}
        for neighbor in neighbors:
            weight = random.choice(weights)
            self.graph[node[0] - self.start_x, node[1] - self.start_y]['neighbors'][neighbor] = weight

    def calculate_heuristic(self, node):
        x1, y1 = node
        return math.dist(node, (self.end_x, self.end_y))

    def is_obstacle(self, node):
        x, y = node
        for obstacle in obstacles:
            if len(obstacle) == 3:  # Circle obstacle (x, y, r)
                ox, oy, r = obstacle
                if math.dist((x, y), (ox, oy)) <= r:
                    return True
            elif len(obstacle) == 4:  # Rectangle obstacle (x1, y1, x2, y2)
                x1, y1, x2, y2 = obstacle
                if x1 <= x <= x2 and y1 <= y <= y2:
                    return True
            elif len(obstacle) == 6:  # Triangle obstacle (x1, y1, x2, y2, x3, y3)
                x1, y1, x2, y2, x3, y3 = obstacle
                # Calculate the three half-space equations
                h1 = (x - x2) * (y1 - y2) - (x1 - x2) * (y - y2)
                h2 = (x - x3) * (y2 - y3) - (x2 - x3) * (y - y3)
                h3 = (x - x1) * (y3 - y1) - (x3 - x1) * (y - y1)
                if h1 >= 0 and h2 >= 0 and h3 >= 0:
                    return True
        return False

    def a_star(self):
        open_list = []
        heapq.heappush(open_list, (0, (self.start_x, self.start_y)))
        closed_list = set()

        while open_list:
            _, current_node = heapq.heappop(open_list)

            if current_node == (self.end_x, self.end_y):
                path = deque()
                while current_node is not None:
                    path.appendleft(current_node)
                    current_node = self.graph[current_node[0] - self.start_x, current_node[1] - self.start_y]['parent']
                return [(float(x), float(y)) for x, y in path]

            if current_node in closed_list:
                continue

            closed_list.add(current_node)
            current_node_data = self.graph[current_node[0] - self.start_x, current_node[1] - self.start_y]

            for neighbor in self.get_neighbors(current_node):
                if self.is_obstacle(neighbor):
                    continue

                weight = current_node_data['neighbors'][neighbor]
                new_distance = self.get_distance(current_node) + weight
                neighbor_distance = self.get_distance(neighbor)
                if new_distance < neighbor_distance:
                    self.set_distance(neighbor, new_distance)
                    self.graph[neighbor[0] - self.start_x, neighbor[1] - self.start_y]['parent'] = current_node
                    priority = (new_distance + self.calculate_heuristic(neighbor), new_distance)
                    heapq.heappush(open_list, (priority, neighbor))

        return None

# print("calculate execution time function...")


# A function to calculate the time of the algorithm execution
def calculate_execution_time(func):
    import time

    def wrapper(*args):
        # for average path planning time metric
        wrapper.total_time += time.time() + wrapper.last_call_time
        wrapper.last_call_time = time.time()
        result = func(*args)
        return result

    wrapper.total_time = 0.0
    wrapper.last_call_time = time.time()
    return wrapper


'''
1. Memory profile decorator to drive an estimation of the memory usage of the implementation
2. the calculate_execution time, which is a user-defined decorator to calculate the execution time of the algorithm implementation
'''
# A function to run the algorithm with 2 decorators:
# print("run a star function...")


@calculate_execution_time
def run_a_star(start_x, start_y, end_x, end_y, weights, obstacles):
    graph = Graph(start_x, start_y, end_x, end_y, weights, obstacles)
    # print("Graph created successfully.")
    path = graph.a_star()
    # print("A* algorithm executed.")
    return path


# Suppose the weights are the time of the journey or any other factor like speed...
weights = [2.135, 1.2364, 3.1367, 4.695, 5.65213, 6.16947, 10.25445, 19.265]
obstacles = create_obstacles()
path = run_a_star(gr.start_x, gr.start_y, gr.end_x, gr.end_y, weights, obstacles)
print(f"Path: {path}")

width = float(gr.end_x) - float(gr.start_x)
height = float(gr.end_y) - float(gr.start_y)
print("This path performs on area of ", width, "m x ", height, "m")
