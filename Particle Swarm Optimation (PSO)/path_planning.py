import numpy as np
import matplotlib.pyplot as plt

# =========================
# ENVIRONMENT
# =========================
class Environment:
    def __init__(self, width, height, robot_radius, start, goal):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = []

    def add_obstacle(self, obs):
        self.obstacles.append(obs)


class Obstacle:
    def __init__(self, center, radius):
        self.center = np.array(center)
        self.radius = radius


# =========================
# PATH GENERATION (SPLINE)
# =========================
def generate_path(env, control_points, resolution):
    points = [env.start]

    for cp in control_points:
        x = cp[0] * env.width
        y = cp[1] * env.height
        points.append([x, y])

    points.append(env.goal)
    points = np.array(points)

    # Linear interpolation (simple spline)
    path = []
    for i in range(len(points) - 1):
        for t in np.linspace(0, 1, resolution // (len(points)-1)):
            p = (1 - t) * points[i] + t * points[i + 1]
            path.append(p)

    return np.array(path)


# =========================
# COST FUNCTION
# =========================
class EnvCostFunction:
    def __init__(self, env, num_control_points, resolution):
        self.env = env
        self.num_control_points = num_control_points
        self.resolution = resolution

    def __call__(self, x):

        # reshape ke control points
        control_points = x.reshape((self.num_control_points, 2))

        path = generate_path(self.env, control_points, self.resolution)

        length = 0
        penalty = 0

        # Hitung panjang path
        for i in range(len(path) - 1):
            length += np.linalg.norm(path[i+1] - path[i])

        # Cek obstacle
        for p in path:
            for obs in self.env.obstacles:
                dist = np.linalg.norm(p - obs.center)
                if dist <= (obs.radius + self.env.robot_radius):
                    penalty += 1000  # penalti besar

        # Cek boundary
        for p in path:
            if not (0 <= p[0] <= self.env.width and 0 <= p[1] <= self.env.height):
                penalty += 1000

        cost = length + penalty

        details = {
            'sol': path,
            'length': length,
            'penalty': penalty
        }

        return cost, details


# =========================
# VISUALIZATION
# =========================
def plot_environment(env):
    plt.xlim(0, env.width)
    plt.ylim(0, env.height)

    # Start & Goal
    plt.scatter(env.start[0], env.start[1], c='green', s=100, label='Start')
    plt.scatter(env.goal[0], env.goal[1], c='red', s=100, label='Goal')

    # Obstacles
    for obs in env.obstacles:
        circle = plt.Circle(obs.center, obs.radius, color='black')
        plt.gca().add_patch(circle)

    plt.legend()


def plot_path(path, color='b'):
    line, = plt.plot(path[:,0], path[:,1], color=color, linewidth=2)
    return line


def update_path(path, line):
    line.set_data(path[:,0], path[:,1])
    plt.pause(0.01)