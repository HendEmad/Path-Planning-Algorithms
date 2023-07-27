import math
from grassfire_algo_without_obstacles import path

path_length = 0
for i in range(len(path) - 1):
    x1, y1 = path[i][0], path[i][1]
    x2, y2 = path[i+1][0], path[i+1][1]
    path_length += math.sqrt((x2-x1) ** 2 + (y2-y1) ** 2)
average_path_length = path_length / (len(path) - 1)

print(f"Average path length: {average_path_length}")