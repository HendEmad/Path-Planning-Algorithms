# create_obstacles.py
import GPSReadings as gr

def create_obstacles():
    obstacles = []

    # circle obstacle
    x = gr.start_x
    y = gr.start_y + 0.12
    r = 0.0165  # Adjusted radius for visibility
    obstacles.append(('circle', x, y, r))

    # another circle obstacle, blue
    x = gr.start_x - 0.1
    y = gr.start_y + 0.18
    r = 0.01  # Adjusted radius for visibility
    obstacles.append(('circle', x, y, r))

    # rectangle obstacle, red
    x1 = gr.start_x + 0.01
    y1 = gr.start_y + 0.01
    x2 = x1 + 0.02
    y2 = y1 + 0.03
    obstacles.append(('rectangle', x1, y1, x2, y2))

    # triangle obstacle, green
    x1 = gr.start_x - 0.07
    y1 = gr.start_y + 0.05
    x2 = x1 + 0.03
    y2 = y1 + 0.01
    x3 = x1 + 0.05
    y3 = y1 + 0.05
    obstacles.append(('triangle', x1, y1, x2, y2, x3, y3))



    # rectangle obstacle, blue
    x1 = gr.start_x - 0.142
    y1 = gr.start_y - 0.01
    x2 = x1 + 0.03
    y2 = y1 + 0.04
    obstacles.append(('rectangle', x1, y1, x2, y2))

    return obstacles

if __name__ == "__main__":
    obstacles = create_obstacles()
    print(obstacles)


