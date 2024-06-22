import server_readings as gr
import heapq


class Graph:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.width = int(end_x) - int(start_x) + 1
        self.height = int(end_y) - int(start_y) + 1
        self.graph = [[{'distance': float('inf')} for j in range(self.height)] for i in range(self.width)]
        self.graph[int(start_x) - int(start_x)][int(start_y) - int(start_y)]['distance'] = 0

    def get_neighbors(self, node):
        x, y = node
        neighbors = [(x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)]
        return [n for n in neighbors if 0 <= n[0] < self.width and 0 <= n[1] < self.height]

    def get_distance(self, node):
        return self.graph[node[0]][node[1]]['distance']

    def set_distance(self, node, distance):
        self.graph[node[0]][node[1]]['distance'] = distance

    def grassfire(self):
        queue = [(0, (int(self.start_x) - int(self.start_x), int(self.start_y) - int(self.start_y)))]
        heapq.heapify(queue)

        while queue:
            current_distance, current = heapq.heappop(queue)
            # current = queue.pop(0)  # queue
            # current_distance = self.get_distance(current)
            if current_distance > self.get_distance(current):
                continue
            neighbors = self.get_neighbors(current)

            for n in neighbors:
                neighbor_distance = self.get_distance(n)
                tentative_distance = current_distance + 1
                if tentative_distance < neighbor_distance:
                    self.set_distance(n, tentative_distance)
                    heapq.heappush(queue, (tentative_distance, n))

        path = [(int(self.end_x) - int(self.start_x), int(self.end_y) - int(self.start_y))]
        current_distance = self.get_distance(path[0])
        current_node = path[0]

        while current_distance > 0:
            neighbors = self.get_neighbors(current_node)

            for n in neighbors:
                neighbor_distance = self.get_distance(n)
                if neighbor_distance < current_distance:
                    current_distance = neighbor_distance
                    current_node = n

            path.append(current_node)

        return [(x + float(self.start_x), y + float(self.start_y)) for x,y in path[::-1]]


def calculate_execution_time(func):
    """
        import time

        def wrapper(*args):
        # for average path planning time metric
            wrapper.total_time += time.time() + wrapper.last_call_time
            wrapper.last_call_time = time.time()
            result = func(*args)
            return result
        wrapper.total_time = 0.0
        wrapper.last_call_time = time.time()
        """

    """
    # for relative standard deviation metric
    import time

    def wrapper(*args):
        start_time = time.time()
        result = func(*args)
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time
    """

    import time

    def wrapper(*args):
        start_time = time.time()
        result = func(*args)
        end_time = time.time()
        execution_time = end_time - start_time
        return result

    return wrapper


@calculate_execution_time
def run_grassfire(start_x, start_y, end_x, end_y):
    graph = Graph(start_x, start_y, end_x, end_y)
    return graph.grassfire()


path = run_grassfire(gr.start_x, gr.start_y, gr.end_x, gr.end_y)
print(f"Path: {path}")

width = float(gr.end_x) - float(gr.start_x) + 1
height = float(gr.end_y) - float(gr.start_y) + 1
print("This path performs on area of ", width, "m x ", height, "m")