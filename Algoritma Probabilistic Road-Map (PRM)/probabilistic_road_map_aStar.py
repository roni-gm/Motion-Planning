import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# parameter
N_SAMPLE = 1000  # number of sample_points
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length

show_animation = True


class Node:
    """
    Node class for A* search
    """

    def __init__(self, x, y, cost, h, parent_index):  # ← tambah h (heuristic)
        self.x = x
        self.y = y
        self.cost = cost
        self.h = h                                     # ← nilai heuristic
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + \
               str(self.cost) + "," + str(self.parent_index)


def heuristic(x, y, gx, gy):                          # ← fungsi heuristic baru
    """Jarak Euclidean dari posisi saat ini ke goal"""
    return math.hypot(gx - x, gy - y)


def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list,
                 robot_radius, *, rng=None):
    """
    Run probabilistic road map planning
    """
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list,
                                         obstacle_y_list)).T)

    sample_x, sample_y = sample_points(start_x, start_y,
                                        goal_x, goal_y, robot_radius,
                                        obstacle_x_list, obstacle_y_list,
                                        obstacle_kd_tree, rng)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_road_map(sample_x, sample_y,
                                  robot_radius, obstacle_kd_tree)

    rx, ry = astar_planning(start_x, start_y, goal_x,  # ← ganti ke astar
                             goal_y, road_map, sample_x, sample_y)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    n_step = round(d / D)

    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True  # collision
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True  # collision

    return False  # OK


def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    """
    Road map generation
    """
    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map


def astar_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    """
    A* path planning pada road map PRM
    Bedanya dengan Dijkstra: pemilihan node pakai cost + heuristic
    sehingga pencarian lebih terarah menuju goal
    """

    # ← Node start: cost=0, h=jarak ke goal
    start_node = Node(sx, sy, 0.0,
                      heuristic(sx, sy, gx, gy), -1)
    # ← Node goal: cost=0, h=0 (sudah di goal)
    goal_node = Node(gx, gy, 0.0, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        # ← PERBEDAAN UTAMA: pilih node dengan cost + h terkecil (bukan cost saja)
        c_id = min(open_set,
                   key=lambda o: open_set[o].cost + open_set[o].h)
        current = open_set[c_id]

        # show graph
        if show_animation and len(closed_set.keys()) % 2 == 0:
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        del open_set[c_id]
        closed_set[c_id] = current

        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)

            # ← Setiap node baru langsung dihitung heuristic-nya ke goal
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d,
                        heuristic(sample_x[n_id], sample_y[n_id], gx, gy),
                        c_id)

            if n_id in closed_set:
                continue
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final course
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover
    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]
            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree, rng):
    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


def main(rng=None):
    print(__file__ + " start!!")

    ox, oy = [], []

    # --- Batas luar: x=-10..60, y=-10..60 ---
    for i in range(-10, 61):
        ox.append(i);   oy.append(-10)
    for i in range(-10, 61):
        ox.append(60);  oy.append(i)
    for i in range(-10, 61):
        ox.append(i);   oy.append(60)
    for i in range(-10, 61):
        ox.append(-10); oy.append(i)

    # --- Dinding vertikal kiri dalam: x=20, y=-10 s/d 40 ---
    for i in range(-10, 41):
        ox.append(20); oy.append(i)

    # --- Dinding vertikal kanan dalam: x=40, y=20 s/d 60 ---
    for i in range(20, 61):
        ox.append(40); oy.append(i)

    # --- Garis horizontal: y=10, x=-10 s/d 10 ---
    for i in range(-10, 11):
        ox.append(i); oy.append(10)

    # --- Garis horizontal: y=30, x=0 s/d 20 ---
    for i in range(0, 21):
        ox.append(i); oy.append(30)

    sx = -5.0
    sy = -5.0
    gx = 50.0
    gy = 50.0
    robot_size = 1.0

    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og", markersize=8)
        plt.plot(gx, gy, "xb", markersize=8, markeredgewidth=2)
        plt.axis("equal")
        plt.grid(True)

    rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size, rng=rng)

    assert rx, 'Cannot found path'

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()