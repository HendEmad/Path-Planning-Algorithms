from ObstaclesGeneration import create_obstacles
import GpsReading as gr
import random
import math
# import cProfile
# import pstats
# from io import StringIO

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

        # Hierarchical Bucketing
        self.high_priority = {}
        self.low_priority = {}

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
            if self.is_obstacle(neighbor):
                weight = random.uniform(1, 20)
            else:
                weight = random.choice(weights)
            self.graph[node]['neighbors'][neighbor] = weight

    def is_obstacle(self, node):
        for obstacle in self.obstacles:
            if obstacle.is_obstacle(node):
                return True
        return False

    # def contract_nodes(self):
    #     contracted_graph = {
    #         (int(self.start_x), int(self.start_y)): self.graph[(int(self.start_x), int(self.start_y))],
    #         (int(self.end_x), int(self.end_y)): self.graph[(int(self.end_x), int(self.end_y))]
    #     }
    #     for node in self.graph:
    #         if node != (int(self.start_x), int(self.start_y)) and node != (int(self.end_x), int(self.end_y)) and self.is_critical_node(node):
    #             contracted_graph[node] = self.graph[node]
    #     # print(f"Contracted graph nodes before setting: {contracted_graph.keys()}")
    #     self.graph = contracted_graph
    #     # print(f"Graph nodes after contraction: {self.graph.keys()}")

    # def is_critical_node(self, node):
    #     start_to_goal_distance = self.euclidean_distance((self.start_x, self.start_y), (self.end_x, self.end_y))
    #     node_to_start_distance = self.euclidean_distance(node, (self.start_x, self.start_y))
    #     node_to_goal_distance = self.euclidean_distance(node, (self.end_x, self.end_y))
    #     if node == (self.start_x, self.start_y) or node == (self.end_x, self.end_y):
    #         return True
    #     return node_to_start_distance + node_to_goal_distance <= start_to_goal_distance * 2.5

    def euclidean_distance(self, node1, node2):
        return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

    def dijkstra(self):
        # self.contract_nodes()  # Contract the graph dynamically

        # Initialize high_priority bucket with the start node
        self.high_priority[0] = [(int(self.start_x), int(self.start_y))]
        while self.high_priority or self.low_priority:
            if self.high_priority:
                min_distance = min(self.high_priority.keys())
                current_node = self.high_priority[min_distance].pop(0)
                if not self.high_priority[min_distance]:
                    del self.high_priority[min_distance]
            else:
                min_distance = min(self.low_priority.keys())
                current_node = self.low_priority[min_distance].pop(0)
                if not self.low_priority[min_distance]:
                    del self.low_priority[min_distance]

            # print(f"Processing node: {current_node}")

            if current_node == (int(self.end_x), int(self.end_y)):
                path = []
                while current_node is not None:
                    path.append(current_node)
                    current_node = self.graph[current_node]['parent']
                path.reverse()
                return [(float(x), float(y)) for x, y in path]

            if min_distance > self.get_distance(current_node):
                continue

            for neighbor in self.get_neighbors(current_node):
                if neighbor not in self.graph:
                    continue

                weight = self.graph[current_node]['neighbors'][neighbor]
                new_distance = self.get_distance(current_node) + weight
                if new_distance < self.get_distance(neighbor):
                    self.set_distance(neighbor, new_distance)
                    self.graph[neighbor]['parent'] = current_node

                    if new_distance < 10:  # Threshold to determine high or low priority
                        if new_distance in self.high_priority:
                            self.high_priority[new_distance].append(neighbor)
                        else:
                            self.high_priority[new_distance] = [neighbor]
                    else:
                        if new_distance in self.low_priority:
                            self.low_priority[new_distance].append(neighbor)
                        else:
                            self.low_priority[new_distance] = [neighbor]

        return None

def calculate_execution_time(func):
    import time

    def wrapper(*args):
        # pr = cProfile.Profile()
        # pr.enable()

        start_time = time.time()
        result = func(*args)
        end_time = time.time()

        # pr.disable()
        # s = StringIO()
        # sortby = 'cumulative'
        # ps = pstats.Stats(pr, stream = s).sort_stats(sortby)
        # ps.print_stats()
        # print(s.getvalue())

        execution_time = end_time - start_time
        print(f"Execution time: {execution_time} seconds")

        return result
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
