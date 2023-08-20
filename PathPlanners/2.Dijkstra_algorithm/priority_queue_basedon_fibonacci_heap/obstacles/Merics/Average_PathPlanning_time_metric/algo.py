import GPS_readings as gr
import random
from heapdict import heapdict
from obstacles import create_obstacles

class Graph:
    def __init__(self, start_x, start_y, end_x, end_y, weights, obstacles):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.graph = {}
        self.width = int(end_x) - int(start_x) + 1
        self.height = int(end_y) - int(start_y) + 1
        self.obstacles = obstacles

        for x in range(int(start_x), int(end_x) + 1):
            for y in range(int(start_y), int(end_y) + 1):
                self.graph[(x, y)] = {'distance': float('inf'),
                                      'parent': None,
                                      'neighbors': {}}

        self.graph[(int(start_x), int(start_y))]['distance'] = 0
        for node in self.graph:
            if node is not None:
                self.set_weights(node, weights)

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        valid_neighbors = []
        for neighbor in neighbors:
            if neighbor in self.graph and self.graph[neighbor] is not None and not self.is_obstacle(neighbor):
                valid_neighbors.append(neighbor)
        return valid_neighbors

    def get_distance(self, node):
        return self.graph[node]['distance']

    def set_distance(self, node, distance):
        self.graph[node]['distance'] = distance

    def set_weights(self, node, weights):
        neighbors = self.get_neighbors(node)
        for neighbor in neighbors:
            weight = random.choice(weights)
            self.graph[node]['neighbors'][neighbor] = weight

    def is_obstacle(self, node):
        for obstacle in self.obstacles:
            if len(obstacle) == 3:
                # circle obstacle
                x, y, r = obstacle
                if (node[0] - x) ** 2 + (node[1] - y) ** 2 <= r ** 2:
                    return True
            elif len(obstacle) == 4:
                # rectangle obstacle
                x1, y1, x2, y2 = obstacle
                if x1 <= node[0] <= x2 and y1 <= node[1] <= y2:
                    return True
            elif len(obstacle) == 6:
                # triangle obstacle
                x1, y1, x2, y2, x3, y3 = obstacle
                A = 1 / 2 * (-y2 * y3 + y1 * (-y2 + y3) + x1 * (x2 - x3) + x2 * y3 - x3 * y2)
                sign = -1 if A < 0 else 1
                s = (y1 * y3 - y1 * y2 - y2 * node[0] + y2 * y3 + node[1] * x2 - node[1] * x3 + node[0] * y2 - node[
                    0] * y3) * sign
                t = (y1 * y2 - y1 * node[0] - y2 * y3 + y3 * node[0] + node[1] * x3 - y3 * node[0]) * sign
                if s > 0 and t > 0 and (s + t) < 2 * A * sign:
                    return True
        return False

    def dijkstra(self):
        heap = heapdict()
        heap[(int(self.start_x), int(self.start_y))] = 0
        while heap:
            current_node, dist = heap.popitem()
            if current_node == (int(self.end_x), int(self.end_y)):
                path = []
                while current_node is not None:
                    path.append(current_node)
                    current_node = self.graph[current_node]['parent']
                path.reverse()
                return [(float(x), float(y)) for x, y in path]

            if dist > self.get_distance(current_node):
                continue

            for neighbor in self.get_neighbors(current_node):
                weight = self.graph[current_node]['neighbors'][neighbor]
                new_distance = self.get_distance(current_node) + weight
                if new_distance < self.get_distance(neighbor):
                    self.set_distance(neighbor, new_distance)
                    self.graph[neighbor]['parent'] = current_node
                    heap[neighbor] = new_distance
        return None


def calculate_execution_time(func):
    import time

    def wrapper(*args):
        start_time = time.time()
        result = func(*args)
        end_time = time.time()

        execution_time = end_time - start_time
        print(f"Execution time: {execution_time} seconds")

        return result, execution_time
    return wrapper


@calculate_execution_time
def run_dijkstra(start_x, start_y, end_x, end_y, weights, obstacles):
    graph = Graph(start_x, start_y, end_x, end_y, weights, obstacles)
    return graph.dijkstra()


weights = [2.135, 1.2364, 3.1367, 4.695, 5.65213, 6.16947, 10.25445, 19.265]
obstacles = create_obstacles()
path = run_dijkstra(gr.start_x, gr.start_y, gr.end_x, gr.end_y, weights, obstacles)
print(f"Path: {path}")

width = float(gr.end_x) - float(gr.start_x)
height = float(gr.end_y) - float(gr.start_y)
print("This path performs on area of ", width, "m x ", height, "m")