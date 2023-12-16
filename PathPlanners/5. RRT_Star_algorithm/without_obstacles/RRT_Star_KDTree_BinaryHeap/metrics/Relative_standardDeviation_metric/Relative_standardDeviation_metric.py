import math
import random
from heapq import heappush, heappop
import numpy as np
import GPS_readings as gr
from scipy.spatial import KDTree
import time


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class RRTStar:
    def __init__(self, start, goal, step_size, max_iter, radius):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.step_size = step_size
        self.max_iter = max_iter
        self.radius = radius
        self.nodes = [self.start]
        self.width = float(gr.end_x) - float(gr.start_x)
        self.height = float(gr.end_y) - float(gr.start_y)
        self.kd_tree = KDTree(np.array([[self.start.x, self.start.y]]))

    def random_position(self):
        x = random.uniform(gr.start_x, gr.end_x)
        y = random.uniform(gr.start_y, gr.end_y)
        return Node(x, y)

    def nearest(self, x_rand):
        '''
        min_dist = float('inf')
        nearest_node = None
        for node in self.nodes:
            dist = math.dist((node.x, node.y), (x_rand.x, x_rand.y))
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node'''
        _, indices = self.kd_tree.query(np.array([[x_rand.x, x_rand.y]]))
        index = indices[0]
        return self.nodes[index]

    def find_neighbors(self, x_new):
        neighbors = []
        for node in self.nodes:
            if math.dist((node.x, node.y), (x_new.x, x_new.y)) <= self.radius:
                neighbors.append(node)
        return neighbors


    def link_nodes(self, x_from, x_to):
        x_from.parent = x_to
        x_from.cost = x_to.cost + math.dist((x_from.x, x_from.y), (x_to.x, x_to.y))
        self.nodes.append(x_from)
        self.kd_tree = KDTree(np.array([[node.x, node.y] for node in self.nodes]))
        print("KD tree data: ", self.kd_tree.data)

    def rrt_star(self):
        for _ in range(self.max_iter):
            x_rand = self.random_position()

            x_nearest = self.nearest(x_rand)

            if math.dist((x_nearest.x, x_nearest.y), (x_rand.x, x_rand.y)) < self.step_size:
                x_new = x_rand
            else:
                theta = math.atan2(x_rand.y - x_nearest.y, x_rand.x - x_nearest.x)
                x_new = Node(x_nearest.x + self.step_size * math.cos(theta),
                             x_nearest.y + self.step_size * math.sin(theta))

            x_neighbors = self.find_neighbors(x_new)
            x_min = x_nearest
            cost_min = x_nearest.cost + math.dist((x_nearest.x, x_nearest.y), (x_new.x, x_new.y))

            for x_near in x_neighbors:
                cost = x_near.cost + math.dist((x_near.x, x_near.y), (x_new.x, x_new.y))
                if cost < cost_min:
                    x_min = x_near
                    cost_min = cost

            self.link_nodes(x_new, x_min)

            for x_near in x_neighbors:
                if x_near != x_min:
                    cost = x_new.cost + math.dist((x_near.x, x_near.y), (x_new.x, x_new.y))
                    if cost < x_near.cost:
                        x_near.parent = x_new
                        x_near.cost = cost

        goal_node = self.nearest(self.goal)
        path = self.reconstruct_path(goal_node)
        return [(float(node.x), float(node.y)) for node in path]

    def reconstruct_path(self, goal):
        path = []
        current = goal
        while current.parent:
            path.append(current)
            current = current.parent
        path.append(current)  # Add the start node
        return path[::-1]  # Reverse the path to start from the beginning


def calculate_execution_time(func):
    import time

    def wrapper(*args):
        start_time = time.time()
        result = func(*args)
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time
    return wrapper


@calculate_execution_time
def run_rrt_star(start_coords, goal_coords):
    rrt_star = RRTStar(start_coords, goal_coords, step_size=5.0, max_iter=5000, radius=20.0)
    path = rrt_star.rrt_star()
    return path


# Example usage
start_coords = (gr.start_x, gr.start_y)
goal_coords = (gr.end_x, gr.end_y)

num_trials = 10
execution_times = []
for _ in range(num_trials):
    start_time = time.time()

    rrt_star = RRTStar(start_coords, goal_coords, step_size=5.0, max_iter=5000, radius=20.0)
    rrt_star_path = rrt_star.rrt_star()

    end_time = time.time()
    execution_time = end_time - start_time
    execution_times.append(execution_time)

    print("RRT* Path:", rrt_star_path)

average_execution_time = sum(execution_times) / num_trials
relative_execution_time = np.std(execution_times) / average_execution_time

print("Average Execution Time:", average_execution_time)
print("Relative Standard Deviation of the average path planning time: ", relative_execution_time)
