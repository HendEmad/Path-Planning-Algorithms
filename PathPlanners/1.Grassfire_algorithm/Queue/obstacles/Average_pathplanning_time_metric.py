from grassfire_algo import run_grassfire, obstacles
from GPS_readings import start_x, start_y, end_x, end_y
import time

total_execution_time = 0.0
num_trials = 10

for i in range(num_trials):
    start_time = time.time()
    run_grassfire(start_x, start_y, end_x, end_y, obstacles)
    end_time = time.time()
    execution_time = end_time - start_time
    total_execution_time += execution_time

average_execution_time = total_execution_time / num_trials
print(f"Average path planning time: {average_execution_time} seconds.")