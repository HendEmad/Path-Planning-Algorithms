# import numpy as np
# import matplotlib.pyplot as plt
# # import GPS_readings as gr
# import GPSReadings as gr
# from obstacles import create_obstacles

# # Environment size in meters
# env_width = 627.92243401797
# env_height = 792.7429830282308

# # Geographic bounds
# min_lon = 149.0
# max_lon = 149.2
# min_lat = -35.4
# max_lat = -35.0

# # Conversion function for geospatial to grid coordinates
# def geo_to_grid(lon, lat, min_lon, max_lon, min_lat, max_lat, width, height):
#     x = (lon - min_lon) / (max_lon - min_lon) * width
#     y = (lat - min_lat) / (max_lat - min_lat) * height
#     return x, y

# # Convert start and goal nodes to grid coordinates
# start_grid = geo_to_grid(gr.start_x, gr.start_y, min_lon, max_lon, min_lat, max_lat, env_width, env_height)
# goal_grid = geo_to_grid(gr.end_x, gr.end_y, min_lon, max_lon, min_lat, max_lat, env_width, env_height)

# # Generate obstacles
# obstacles = create_obstacles()

# # Convert obstacles to grid coordinates
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

# # Create a plot
# fig, ax = plt.subplots(figsize=(10, 12))

# # Plot the environment boundaries
# plt.xlim(0, env_width)
# plt.ylim(0, env_height)

# # Plot the start and goal nodes
# plt.plot(start_grid[0], start_grid[1], 'go', label='Start Node')
# plt.plot(goal_grid[0], goal_grid[1], 'ro', label='Goal Node')

# # Plot the obstacles
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

# # Add labels and title
# plt.xlabel('Width (m)')
# plt.ylabel('Height (m)')
# plt.title('Small environment without obstacles')
# plt.legend()

# # Adding grid lines for better visualization
# plt.grid(which='both', color='grey', linestyle='-', linewidth=0.5)
# plt.xticks(np.arange(0, env_width, 50))
# plt.yticks(np.arange(0, env_height, 50))

# # Show the plot
# plt.show()


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import GpsReading as gr

# Environment size in meters
env_width = 990.9186170273956
env_height =  1177.7359179099876

# Geographic bounds
min_lon = 149.0
max_lon = 149.2
min_lat = -35.4
max_lat = -35.0

# Conversion function for geospatial to grid coordinates
'''
(lon - min_lon) / (max_lon - min_lon) && (lat - min_lat) / (max_lat - min_lat) --> normalizing the longitude and latitude to a range between 0 and 1
then, multiplying bt width and height to scale these normalized values to the dimensions of the grid.
'''
def geo_to_grid(lon, lat, min_lon, max_lon, min_lat, max_lat, width, height):
    x = (lon - min_lon) / (max_lon - min_lon) * width
    y = (lat - min_lat) / (max_lat - min_lat) * height
    return x, y

# Convert start and goal nodes to grid coordinates
start_grid = geo_to_grid(gr.start_x, gr.start_y, min_lon, max_lon, min_lat, max_lat, env_width, env_height)
goal_grid = geo_to_grid(gr.end_x, gr.end_y, min_lon, max_lon, min_lat, max_lat, env_width, env_height)
print(f"start node = {start_grid}, goal node = {goal_grid}")

# Paths to obstacle images
image_paths = {
    'car': 'images/car_image.png',             # Replace with actual paths
    'rock': 'images/rock_image.png',
    'traffic_light': 'images/traffic_light_image.png',
    'tree': 'images/tree_image.png',
    'pedestrian': 'images/pedestrian_image.png'
}

# Define obstacle positions (longitude, latitude) relative to the environment
# Adjusting the positions to ensure they are within the environment grid dimensions and do not overlap
obstacle_positions = [
    ('tree', gr.start_x + 0.02, gr.start_y + 0.1),       # Trees on one side of the street
    ('tree', gr.start_x + 0.02, gr.start_y + 0.15),
    ('tree', gr.start_x + 0.02, gr.start_y + 0.2),
    ('tree', gr.start_x + 0.02, gr.start_y + 0.25),
    ('car', gr.start_x - 0.044, gr.start_y ),       # Parked cars on the other side
    ('car', gr.start_x - 0.09, gr.start_y + 0.15),
    ('car', gr.start_x - 0.086, gr.start_y),
    ('traffic_light', gr.start_x - 0.02, gr.start_y + 0.08), # Traffic light near the start
    ('rock', gr.start_x - 0.08, gr.start_y + 0.12),      # Random obstacles
    ('rock', gr.start_x - 0.0999, gr.start_y + 0.18),
    ('pedestrian', gr.start_x - 0.056, gr.start_y + 0.1),  # Pedestrians walking
    ('pedestrian', gr.start_x - 0.016, gr.start_y + 0.15)
]

def create_obstacles():
        from EnvVisualization import obstacle_positions
        obstacles = []
        for obs in obstacle_positions:
            if obs[0] == 'tree' or obs[0] == 'traffic_light':
                obstacles.append((obs[1], obs[2], 0.02))
            elif obs[0] == 'car':
                obstacles.append((obs[1] - 0.01, obs[2] - 0.02, obs[1] + 0.01, obs[2]+ 0.02))
            elif obs[0] == 'rock':
                obstacles.append((obs[1], obs[2], 0.01))
            elif obs[0] == 'pedestrian':
                obstacles.append((obs[1] - 0.005, obs[2] - 0.005, obs[1] + 0.005, obs[2] + 0.005))
        return obstacles

# Load image function
def load_image(image_path, zoom=0.1):
    img = mpimg.imread(image_path)
    return OffsetImage(img, zoom=zoom)

# Convert obstacle positions to grid coordinates and load images
grid_obstacles = []
for obs in obstacle_positions:
    x, y = geo_to_grid(obs[1], obs[2], min_lon, max_lon, min_lat, max_lat, env_width, env_height)
    # zoom_level = 0.2 if obs[0] == 'tree' else 0.15  # Increase zoom level for better visibility
    if(obs[0] == 'tree'):
        zoom_level = 0.2
    elif(obs[0] == 'car'):
        zoom_level = 0.07
    else:
        zoom_level = 0.15

    img = load_image(image_paths[obs[0]], zoom=zoom_level)
    grid_obstacles.append((img, x, y))
    print(f"Obstacle: {obs[0]}, Grid Position: ({x}, {y})")  # Debug print

# Create a plot
fig, ax = plt.subplots(figsize=(12, 10))

# Plot the environment boundaries
plt.xlim(0, env_width)
plt.ylim(0, env_height)

# Plot the start and goal nodes
plt.plot(start_grid[0], start_grid[1], 'go', label='Start Node')
plt.plot(goal_grid[0], goal_grid[1], 'ro', label='Goal Node')

# Plot the obstacles with images
for obstacle in grid_obstacles:
    img_box = AnnotationBbox(obstacle[0], (obstacle[1], obstacle[2]), frameon=False)
    ax.add_artist(img_box)

# Add labels and title
plt.xlabel('Width (m)')
plt.ylabel('Height (m)')
plt.title('Realistic Street Environment with Obstacles')
plt.legend()

# Adding grid lines for better visualization
plt.grid(which='both', color='grey', linestyle='-', linewidth=0.5)
plt.xticks(np.arange(0, env_width, 50))
plt.yticks(np.arange(0, env_height, 50))

# Show the plot
plt.show()
