import heapq
import math
from collections import deque
import random
import numpy as np
from obstacles import create_obstacles
import GPS_readings as gr
from heapdict import heapdict

# PRM Parameters
N_SAMPLES = 100  # Number of random samples
K_NEAREST = 5  # Number of nearest neighbors to consider
MAX_DISTANCE = 100  # Maximum distance for connecting edges


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
            if neighbor not in self.weights:
                self.weights[neighbor] = {}
            weight = random.choice(weights)
            self.graph[node[0] - self.start_x, node[1] - self.start_y]['neighbors'][neighbor] = weight
            self.weights[node][neighbor] = weight
            self.weights[neighbor][node] = weight

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

    def prm(self):
        roadmap = []
        nodes = []

        # Generate random samples
        for _ in range(N_SAMPLES):
            x = random.randint(self.start_x, self.end_x)
            y = random.randint(self.start_y, self.end_y)
            if not self.is_obstacle((x, y)):
                nodes.append((x, y))

        # Set weights for all nodes
        for node in nodes:
            self.set_weights(node, weights)

        # Connect edges between nearest neighbors
        for node in nodes:
            distances = [(math.dist(node, other), other) for other in nodes if other != node]
            distances.sort()
            for i in range(min(K_NEAREST, len(distances))):
                distance, neighbor = distances[i]
                if distance <= MAX_DISTANCE:
                    roadmap.append((node, neighbor))

        return roadmap

    def a_star(self):
        start = (self.start_x, self.start_y)
        end = (self.end_x, self.end_y)
        open_list = heapdict({start:self.get_distance(start)})
        closed_list = set()

        while open_list:
            current, current_distance = open_list.popitem()
            if current == end:
                path = self.reconstruct_path(current)
                return path
            closed_list.add(current)

            neighbors = self.get_neighbors(current)
            for neighbor in neighbors:
                if neighbor in closed_list:
                    continue
                tentative_distance = self.get_distance(current) + self.weights[current][neighbor]
                if neighbor not in open_list or tentative_distance < self.get_distance(neighbor):
                    self.set_distance(neighbor, tentative_distance)
                    open_list[neighbor] = tentative_distance + self.calculate_heuristic(neighbor)
                    self.graph[neighbor[0] - self.start_x, neighbor[1] - self.start_y]['parent'] = current

        return None

    def reconstruct_path(self, node):
        path = deque()
        while node is not None:
            path.appendleft(node)
            node = self.graph[node[0] - self.start_x, node[1] - self.start_y]['parent']
        return list(path)


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


# Example usage
start_x = gr.start_x
start_y = gr.start_y
end_x = gr.end_x
end_y = gr.end_y
weights = [2.135, 1.2364, 3.1367, 4.695, 5.65213, 6.16947, 10.25445, 19.265]
obstacles = create_obstacles()

path = run_a_star(start_x, start_y, end_x, end_y, weights, obstacles)
print("Path: ", path)