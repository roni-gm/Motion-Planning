import matplotlib.pyplot as plt
import math

show_animation = True


class AStar:
    def __init__(self, ox, oy, resolution, robot_radius):

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost  # g cost (actual cost from start)
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A* path search
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)

        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        
        # Untuk A*, kita perlu menyimpan f_cost = g_cost + h_cost
        # Tapi kita bisa menghitung h_cost saat membandingkan
        open_set[self.calc_index(start_node)] = start_node

        while True:
            # Cari node dengan f_cost terendah (g_cost + heuristic)
            c_id = min(open_set, 
                      key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            
            current = open_set[c_id]

            if show_animation:
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")

                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])

                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # Jika sampai di goal
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Pindahkan dari open_set ke closed_set
            del open_set[c_id]
            closed_set[c_id] = current

            # Eksplorasi tetangga
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)

                n_id = self.calc_index(node)

                # Jika sudah di closed_set, lewati
                if n_id in closed_set:
                    continue

                # Jika node tidak valid, lewati
                if not self.verify_node(node):
                    continue

                # Jika node baru ditemukan
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    # Jika node sudah ada di open_set, update jika cost lebih rendah
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        # Buat jalur akhir
        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_heuristic(self, goal_node, current_node):
        """
        Menghitung heuristic (perkiraan jarak ke goal)
        Menggunakan Euclidean distance
        """
        # Konversi indeks grid ke koordinat sebenarnya
        gx = self.calc_position(goal_node.x, self.min_x)
        gy = self.calc_position(goal_node.y, self.min_y)
        cx = self.calc_position(current_node.x, self.min_x)
        cy = self.calc_position(current_node.y, self.min_y)
        
        # Euclidean distance sebagai heuristic
        h = math.hypot(gx - cx, gy - cy)
        
        return h

    def calc_final_path(self, goal_node, closed_set):
        rx = [self.calc_position(goal_node.x, self.min_x)]
        ry = [self.calc_position(goal_node.y, self.min_y)]

        parent_index = goal_node.parent_index

        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        # Balik urutan agar dari start ke goal
        rx.reverse()
        ry.reverse()
        
        return rx, ry

    def calc_position(self, index, minp):
        return index * self.resolution + minp

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]

        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)]
        ]
        return motion


def main():
    print(__file__ + " start!!")

    sx = -5.0
    sy = -5.0
    gx = 50.0
    gy = 50.0

    grid_size = 2.0
    robot_radius = 1.0

    ox, oy = [], []

    # =============================
    # WALL (boundary map)
    # =============================
    for i in range(-10, 61):
        ox.append(i)
        oy.append(-10)

    for i in range(-10, 61):
        ox.append(60)
        oy.append(i)

    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)

    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)

    # Vertical wall (x=20)
    for i in range(-10, 41):
        ox.append(20)
        oy.append(i)

    # Horizontal wall (y=30)
    for i in range(0, 21):
        ox.append(i)
        oy.append(30)

    # Horizontal wall (y=10)
    for i in range(-10, 11):
        ox.append(i)
        oy.append(10)

    # Vertical wall (x=40)
    for i in range(20, 61):
        ox.append(40)
        oy.append(i)

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")
        plt.title("A* Path Planning")

    # Menggunakan AStar, bukan Dijkstra
    a_star = AStar(ox, oy, grid_size, robot_radius)

    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:
        plt.plot(rx, ry, "-r", linewidth=2, label="A* Path")
        plt.legend()
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()