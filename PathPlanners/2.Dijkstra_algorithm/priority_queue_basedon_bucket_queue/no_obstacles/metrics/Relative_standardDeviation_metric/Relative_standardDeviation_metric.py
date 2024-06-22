from algo import run_dijkstra, weights
from server_readings import start_x, start_y, end_x, end_y
import numpy as np

num_trials = 10
execution_times = []

for i in range(num_trials):
    result, execution_time = run_dijkstra(start_x, start_y, end_x, end_y, weights)
    execution_times.append(execution_time)

average_execution_time = sum(execution_times) / num_trials

relative_std_dev = np.std(execution_times) / average_execution_time
print(f"Relative standard deviation of the average path planning time: {relative_std_dev}")