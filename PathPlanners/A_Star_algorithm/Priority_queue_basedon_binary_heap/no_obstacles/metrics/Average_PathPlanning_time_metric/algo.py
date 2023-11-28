import heapq
import math
from collections import deque
import GPS_readings as gr
import random
import numpy as np


class Graph:
    def __init__(self, start_x, start_y, end_x, end_y, weights):
        self.start_x = int(start_x)
        self.start_y = int(start_y)
        self.end_x = int(end_x)
        self.end_y = int(end_y)
        self.width = self.end_x - self.start_x + 1
        self.height = self.end_y - self.start_y + 1
        self.weights = {}  # weights[node1][node] = the weight between node 1 and node 2
        self.graph = np.empty((self.width, self.height), dtype=object)

        for x in range(self.start_x, self.end_x + 1):
            for y in range(self.start_y, self.end_y + 1):
                self.graph[x - self.start_x, y - self.start_y] = {'distance': math.inf,
                                                                  'parent': None,
                                                                  'neighbors': {}}

        self.graph[0, 0]['distance'] = 0
        for x in range(self.start_x, self.end_x + 1):
            for y in range(self.start_y, self.end_y + 1):
                self.set_weights((x, y), weights)

    # print("get neighbors function...")

    # A function to get the neighbors of each node
    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        return [(nx, ny) for nx, ny in neighbors if
                0 <= nx - self.start_x < self.width and 0 <= ny - self.start_y < self.height]

    # print("get distance function...")

    # A function to get the distance of the current node
    def get_distance(self, node):
        x, y = node
        return self.graph[x - self.start_x, y - self.start_y]['distance']

    # print("set distance function...")

    # A function to set the distance of the current node
    def set_distance(self, node, distance):
        x, y = node
        self.graph[x - self.start_x, y - self.start_y]['distance'] = distance

    # print("set weights function...")

    # A function to choose the weights of the graph segments/edges randomly
    def set_weights(self, node, weights):
        neighbors = self.get_neighbors(node)
        self.weights[node] = {}
        for neighbor in neighbors:
            weight = random.choice(weights)
            self.graph[node[0] - self.start_x, node[1] - self.start_y]['neighbors'][neighbor] = weight

    # print("calculate heuristic function...")

    # A function to calculate the euclidean distance from the current node to the target node
    def calculate_heuristic(self, node):
        x1, y1 = node
        return math.dist(node, (self.end_x, self.end_y))

    # print("A star function...")

    # A function of the A* algorithm
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
                    # print("Current path: ", [(float(x), float(y)) for x, y in path])
                    current_node = self.graph[current_node[0] - self.start_x, current_node[1] - self.start_y]['parent']
                return [(float(x), float(y)) for x, y in path]

            if current_node in closed_list:
                continue

            closed_list.add(current_node)
            current_node_data = self.graph[current_node[0] - self.start_x, current_node[1] - self.start_y]

            for neighbor in self.get_neighbors(current_node):
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
def run_a_star(start_x, start_y, end_x, end_y, weights):
    graph = Graph(start_x, start_y, end_x, end_y, weights)
    # print("Graph created successfully.")
    path = graph.a_star()
    # print("A* algorithm executed.")
    return path


# Suppose the weights are the time of the journey or any other factor like speed...
weights = [2.135, 1.2364, 3.1367, 4.695, 5.65213, 6.16947, 10.25445, 19.265]
path = run_a_star(gr.start_x, gr.start_y, gr.end_x, gr.end_y, weights)
print(f"Path: {path}")

width = float(gr.end_x) - float(gr.start_x)
height = float(gr.end_y) - float(gr.start_y)
print("This path performs on area of ", width, "m x ", height, "m")
