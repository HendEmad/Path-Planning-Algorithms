import numpy as np
import GPS_readings as gr


def create_obstacles():
    obstacles = []

    # circle obstacle
    x = (gr.end_x + gr.start_x)/2
    y = (gr.end_y + gr.start_y)/2
    r = 3
    obstacles.append((x, y, r))

    # rectangle obstacle
    x1 = gr.start_x + 100
    y1 = gr.start_y + 100
    x2 = x1 + 5
    y2 = y1 + 8
    obstacles.append((x1, y1, x2, y2))

    # triangle obstacle
    x1 = gr.start_x + 500
    y1 = gr.start_y + 500
    x2 = x1 + 5
    y2 = y1 + 5
    x3 = (x1 + x2) / 2
    y3 = (y1 + y2) / 2
    obstacles.append((x1, y1, x2, y2, x3, y3))

    # another circle obstacle
    x = gr.start_x + 400
    y = gr.start_y + 400
    r = 3
    obstacles.append((x, y, r))

    # rectangle obstacle
    x1 = gr.start_x + 600
    y1 = gr.start_y + 650
    x2 = x1 + 5
    y2 = y1 + 8
    obstacles.append((x1, y1, x2, y2))

    return obstacles


if __name__ == "__main__":
    obstacles = create_obstacles()
    print(obstacles)