from algo import run_dijkstra, weights, calculate_execution_time
from GPS_readings import start_x, start_y, end_x, end_y

total_execution_time = 0
num_trials = 10
for i in range(num_trials):
    execution_time = calculate_execution_time(run_dijkstra)(start_x, start_y, end_x, end_y, weights)
    total_execution_time += execution_time

avg_execution_time = total_execution_time / num_trials
print(f"Average execution time for {num_trials} trials: {avg_execution_time} seconds")