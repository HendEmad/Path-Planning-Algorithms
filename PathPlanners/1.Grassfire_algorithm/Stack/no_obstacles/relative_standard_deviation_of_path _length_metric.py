import math
from grassfire_algo_without_obstacles import path

path_length = 0
path_diffs = []
for i in range(len(path) - 1):
    x1, y1 = path[i]
    x2, y2 = path[i+1]
    diff = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    path_length += diff
    path_diffs.append(diff)
average_path_length = path_length / (len(path) - 1)
std_dev_path_length = math.sqrt(sum([(diff - average_path_length) ** 2 for diff in path_diffs]) / (len(path_diffs) - 1))

print(f"relative standard deviation of path length: {std_dev_path_length}")