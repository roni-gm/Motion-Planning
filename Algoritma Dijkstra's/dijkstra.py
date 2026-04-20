import matplotlib.pyplot as plt
import math
import heapq
import itertools

show_animation = True

class Dijkstra:
    def __init__(self, ox, oy, resolution, robot_radius):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        counter = itertools.count()
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set = []
        heapq.heappush(open_set, (0, next(counter), start_node))
        closed_set = {}

        while open_set:
            _, _, current = heapq.heappop(open_set)
            c_id = self.calc_index(current)

            if c_id in closed_set:
                continue

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            closed_set[c_id] = current

            for dx, dy, move_cost in self.motion:
                node = self.Node(current.x + dx, current.y + dy,
                                 current.cost + move_cost, c_id)
                if not self.verify_node(node):
                    continue
                heapq.heappush(open_set, (node.cost, next(counter), node))

        return self.calc_final_path(goal_node, closed_set)

    def calc_final_path(self, goal_node, closed_set):
        rx = [self.calc_position(goal_node.x, self.min_x)]
        ry = [self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent_index
        while parent != -1:
            n = closed_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent_index
        return rx, ry

    def calc_position(self, index, minp):
        return index * self.resolution + minp

    def calc_xy_index(self, pos, minp):
        return round((pos - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    if math.hypot(iox - x, ioy - y) <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, 1.414], [-1, 1, 1.414], [1, -1, 1.414], [1, 1, 1.414]]

class AStar(Dijkstra):
    def planning(self, sx, sy, gx, gy):
        counter = itertools.count()
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)
        open_set = []
        heapq.heappush(open_set, (0, next(counter), start_node))
        closed_set = {}

        while open_set:
            _, _, current = heapq.heappop(open_set)
            c_id = self.calc_index(current)
            if c_id in closed_set:
                continue
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            closed_set[c_id] = current
            for dx, dy, move_cost in self.motion:
                node = self.Node(current.x + dx, current.y + dy,
                                 current.cost + move_cost, c_id)
                if not self.verify_node(node):
                    continue
                priority = node.cost + self.calc_heuristic(goal_node, node)
                heapq.heappush(open_set, (priority, next(counter), node))
        return self.calc_final_path(goal_node, closed_set)

    @staticmethod
    def calc_heuristic(n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

class DStar(Dijkstra):
    pass

def main():
    # Posisi Start dan Goal sesuai gambar
    sx, sy = -5.0, -5.0
    gx, gy = 50.0, 50.0
    grid_size = 1.0
    robot_radius = 1.0

    ox, oy = [], []

    # 1. Frame Luar (Boundary)
    for i in range(-10, 61):
        ox.append(i); oy.append(-10) # Bawah
        ox.append(i); oy.append(60)  # Atas
        ox.append(-10); oy.append(i) # Kiri
        ox.append(60); oy.append(i)  # Kanan

    # 2. Obstacle sesuai Gambar 5-3 (Dinding Penyekat)
    # Dinding vertikal pertama (x=20)
    for i in range(-10, 40):
        ox.append(20); oy.append(i)
        
    # Dinding horizontal tengah (y=30)
    for i in range(0, 21):
        ox.append(i); oy.append(30)
        
    # Dinding horizontal bawah (y=10)
    for i in range(-10, 11):
        ox.append(i); oy.append(10)
        
    # Dinding vertikal kedua (x=40)
    for i in range(20, 61):
        ox.append(40); oy.append(i)

    # Visualisasi: hanya Dijkstra untuk sekarang
    plt.figure(figsize=(8, 8))
    plt.title("Dijkstra")
    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.plot(rx, ry, "-r")
    plt.grid(True)
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    margin = 1
    ax.set_xlim(min(ox) - margin, max(ox) + margin)
    ax.set_ylim(min(oy) - margin, max(oy) + margin)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()