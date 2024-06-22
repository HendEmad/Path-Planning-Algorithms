import math
import random
import numpy as np
from scipy.spatial import distance
import server_readings as gr


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
        self.nodes = {(self.start.x, self.start.y): self.start}
        self.width = float(gr.end_x) - float(gr.start_x)
        self.height = float(gr.end_y) - float(gr.start_y)

    def random_position(self):
        x = random.uniform(gr.start_x, gr.end_x)
        y = random.uniform(gr.start_y, gr.end_y)
        return Node(x, y)

    def nearest(self, x_rand):
        min_dist = float('inf')
        nearest_node = None

        for coord, node in self.nodes.items():
            dist = distance.euclidean((node.x, node.y), (x_rand.x, x_rand.y))
            if dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node

    def find_neighbors(self, x_new):
        neighbors = []

        for coord, node in self.nodes.items():
            if distance.euclidean((node.x, node.y), (x_new.x, x_new.y)) <= self.radius:
                neighbors.append(node)

        return neighbors

    def link_nodes(self, x_from, x_to):
        x_from.parent = x_to
        x_from.cost = x_to.cost + distance.euclidean((x_from.x, x_from.y), (x_to.x, x_to.y))
        self.nodes[(x_from.x, x_from.y)] = x_from

    def rrt_star(self):
        for _ in range(self.max_iter):
            x_rand = self.random_position()

            x_nearest = self.nearest(x_rand)

            if distance.euclidean((x_nearest.x, x_nearest.y), (x_rand.x, x_rand.y)) < self.step_size:
                x_new = x_rand
            else:
                theta = math.atan2(x_rand.y - x_nearest.y, x_rand.x - x_nearest.x)
                x_new = Node(x_nearest.x + self.step_size * math.cos(theta),
                             x_nearest.y + self.step_size * math.sin(theta))

            x_neighbors = self.find_neighbors(x_new)
            x_min = x_nearest
            cost_min = x_nearest.cost + distance.euclidean((x_nearest.x, x_nearest.y), (x_new.x, x_new.y))

            for x_near in x_neighbors:
                cost = x_near.cost + distance.euclidean((x_near.x, x_near.y), (x_new.x, x_new.y))
                if cost < cost_min:
                    x_min = x_near
                    cost_min = cost

            self.link_nodes(x_new, x_min)

            for x_near in x_neighbors:
                if x_near != x_min:
                    cost = x_new.cost + distance.euclidean((x_near.x, x_near.y), (x_new.x, x_new.y))
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
        # for average path planning time metric
        wrapper.total_time += time.time() - wrapper.last_call_time
        wrapper.last_call_time = time.time()
        result = func(*args)
        return result

    wrapper.total_time = 0.0
    wrapper.last_call_time = time.time()
    return wrapper


@calculate_execution_time
def run_rrt_star(start_x, start_y, end_x, end_y):
    graph = RRTStar((start_x, start_y), (end_x, end_y), step_size=5.0, max_iter=5000, radius=20.0)
    path = graph.rrt_star()
    return path


# Example usage for 10 trials
num_trials = 10
for _ in range(num_trials):
    start_coords = (gr.start_x, gr.start_y)
    goal_coords = (gr.end_x, gr.end_y)
    rrt_star_path = run_rrt_star(*start_coords, *goal_coords)

average_time = run_rrt_star.total_time / num_trials
print(f"Average RRT* Path Planning Time for {num_trials} trials: {average_time:.4f} seconds")
