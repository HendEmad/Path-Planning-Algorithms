import GpsReading as gr

class Obstacle:
    def __init__(self, obstacle_type, x, y, radius_or_width, height=None):
        self.obstacle_type = obstacle_type
        self.x = x
        self.y = y
        self.radius_or_width = radius_or_width
        self.height = height

    def is_obstacle(self, point):
        if self.obstacle_type in ['tree', 'traffic_light', 'rock']:
            # Circular obstacle
            distance = ((point[0] - self.x) ** 2 + (point[1] - self.y) ** 2) ** 0.5
            return distance <= self.radius_or_width
        elif self.obstacle_type in ['car', 'pedestrian']:
            # Rectangular obstacle
            return (
                self.x - self.radius_or_width / 2 <= point[0] <= self.x + self.radius_or_width / 2
                and self.y - self.height / 2 <= point[1] <= self.y + self.height / 2
            )
        return False

# Define obstacle positions (longitude, latitude) relative to the environment
# Adjusting the positions to ensure they are within the environment grid dimensions and do not overlap
obstacle_positions = [
    ('tree', gr.start_x + 0.02, gr.start_y + 0.1),       # Trees on one side of the street
    ('tree', gr.start_x + 0.02, gr.start_y + 0.15),
    ('tree', gr.start_x + 0.02, gr.start_y + 0.2),
    ('tree', gr.start_x + 0.02, gr.start_y + 0.25),
    ('car', gr.start_x - 0.044, gr.start_y),       # Parked cars on the other side
    ('car', gr.start_x - 0.09, gr.start_y + 0.15),
    ('car', gr.start_x - 0.086, gr.start_y),
    ('traffic_light', gr.start_x - 0.02, gr.start_y + 0.08), # Traffic light near the start
    ('rock', gr.start_x - 0.08, gr.start_y + 0.12),      # Random obstacles
    ('rock', gr.start_x - 0.0999, gr.start_y + 0.18),
    ('pedestrian', gr.start_x - 0.056, gr.start_y + 0.1),  # Pedestrians walking
    ('pedestrian', gr.start_x - 0.016, gr.start_y + 0.15)
]

def create_obstacles():
    obstacles = []
    for obs in obstacle_positions:
        if obs[0] in ['tree', 'traffic_light', 'rock']:
            obstacles.append(Obstacle(obs[0], obs[1], obs[2], 0.02))
        elif obs[0] == 'car':
            obstacles.append(Obstacle(obs[0], obs[1], obs[2], 0.02, 0.04))
        elif obs[0] == 'pedestrian':
            obstacles.append(Obstacle(obs[0], obs[1], obs[2], 0.01, 0.01))
    return obstacles
