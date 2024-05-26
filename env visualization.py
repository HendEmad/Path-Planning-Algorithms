import numpy as np
import matplotlib.pyplot as plt
# import GPS_readings as gr
import GPSReadings as gr
# from obstacles import create_obstacles

# Environment size in meters
env_width = 627.92243401797
env_height = 792.7429830282308

# Geographic bounds
min_lon = 149.0
max_lon = 149.2
min_lat = -35.4
max_lat = -35.0

# Conversion function for geospatial to grid coordinates
def geo_to_grid(lon, lat, min_lon, max_lon, min_lat, max_lat, width, height):
    x = (lon - min_lon) / (max_lon - min_lon) * width
    y = (lat - min_lat) / (max_lat - min_lat) * height
    return x, y

# Convert start and goal nodes to grid coordinates
start_grid = geo_to_grid(gr.start_x, gr.start_y, min_lon, max_lon, min_lat, max_lat, env_width, env_height)
goal_grid = geo_to_grid(gr.end_x, gr.end_y, min_lon, max_lon, min_lat, max_lat, env_width, env_height)

# Generate obstacles
# obstacles = create_obstacles()

# Convert obstacles to grid coordinates
# grid_obstacles = []
# for obs in obstacles:
#     if obs[0] == 'circle':  # Circle
#         x, y = geo_to_grid(obs[1], obs[2], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
#         grid_obstacles.append(('circle', x, y, obs[3] * (env_width / (max_lon - min_lon))))  # Scale radius
#     elif obs[0] == 'rectangle':  # Rectangle
#         x1, y1 = geo_to_grid(obs[1], obs[2], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
#         x2, y2 = geo_to_grid(obs[3], obs[4], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
#         grid_obstacles.append(('rectangle', x1, y1, x2, y2))
#     elif obs[0] == 'triangle':  # Triangle
#         x1, y1 = geo_to_grid(obs[1], obs[2], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
#         x2, y2 = geo_to_grid(obs[3], obs[4], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
#         x3, y3 = geo_to_grid(obs[5], obs[6], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
#         grid_obstacles.append(('triangle', x1, y1, x2, y2, x3, y3))

# Create a plot
fig, ax = plt.subplots(figsize=(10, 12))

# Plot the environment boundaries
plt.xlim(0, env_width)
plt.ylim(0, env_height)

# Plot the start and goal nodes
plt.plot(start_grid[0], start_grid[1], 'go', label='Start Node')
plt.plot(goal_grid[0], goal_grid[1], 'ro', label='Goal Node')

# Plot the obstacles
# for obstacle in grid_obstacles:
#     if obstacle[0] == 'circle':
#         circle = plt.Circle((obstacle[1], obstacle[2]), obstacle[3], color='blue', fill=False)
#         ax.add_patch(circle)
#     elif obstacle[0] == 'rectangle':
#         width = obstacle[3] - obstacle[1]
#         height = obstacle[4] - obstacle[2]
#         rect = plt.Rectangle((obstacle[1], obstacle[2]), width, height, color='red', fill=False)
#         ax.add_patch(rect)
#     elif obstacle[0] == 'triangle':
#         triangle = plt.Polygon([(obstacle[1], obstacle[2]), (obstacle[3], obstacle[4]), (obstacle[5], obstacle[6])], color='green', fill=False)
#         ax.add_patch(triangle)

# Add labels and title
plt.xlabel('Width (m)')
plt.ylabel('Height (m)')
plt.title('Small environment without obstacles')
plt.legend()

# Adding grid lines for better visualization
plt.grid(which='both', color='grey', linestyle='-', linewidth=0.5)
plt.xticks(np.arange(0, env_width, 50))
plt.yticks(np.arange(0, env_height, 50))

# Show the plot
plt.show()
