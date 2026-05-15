import numpy as np
import matplotlib.pyplot as plt
import datetime

# ==========================================
# GENETIC ALGORITHM FOR MOTION PLANNING
# ==========================================

# Environment map
GRID_SIZE = 10

# Start and Goal Position
START = (0, 0)
GOAL = (9, 9)

# Obstacles
OBSTACLES = [
    (3, 3), (3, 4), (3, 5),
    (4, 5), (5, 5), (6, 5),
    (7, 5), (7, 6), (7, 7)
]

# GA Parameters
POPULATION_SIZE = 20
GENE_LENGTH = 25
MUTATION_RATE = 0.1
MAX_GENERATION = 200

# Possible robot movements
MOVES = ['U', 'D', 'L', 'R']

# ==========================================
# CREATE RANDOM GENE
# ==========================================

def create_gene(length):
    return ''.join(np.random.choice(MOVES, length))

# ==========================================
# MOVE ROBOT
# ==========================================

def move(position, action):
    x, y = position

    if action == 'U':
        y += 1
    elif action == 'D':
        y -= 1
    elif action == 'L':
        x -= 1
    elif action == 'R':
        x += 1

    # Boundary checking
    x = max(0, min(GRID_SIZE - 1, x))
    y = max(0, min(GRID_SIZE - 1, y))

    return (x, y)

# ==========================================
# FITNESS FUNCTION (combined cost -> higher fitness better)
# ==========================================

def calculate_fitness(gene):
    position = START
    collisions = 0
    steps_taken = 0
    stagnant = 0

    for step in gene:
        next_position = move(position, step)

        # If move doesn't change position (boundary) count as stagnant
        if next_position == position:
            stagnant += 1
            continue

        steps_taken += 1

        # Collision detection
        if next_position in OBSTACLES:
            collisions += 1
            # do not move into obstacle
        else:
            position = next_position

    # Manhattan distance to goal
    distance = abs(position[0] - GOAL[0]) + abs(position[1] - GOAL[1])

    # Combine costs (lower is better) with weights
    w_dist = 1.0
    w_len = 0.1
    w_col = 5.0
    w_stag = 0.5

    cost = w_dist * distance + w_len * steps_taken + w_col * collisions + w_stag * stagnant

    # Convert to fitness (higher is better)
    fitness = 1.0 / (1.0 + cost)

    # Large bonus if goal reached (make fitness dominant)
    if position == GOAL:
        fitness += 10.0

    return fitness, position

# ==========================================
# CREATE POPULATION
# ==========================================

def create_population():
    population = {}

    for _ in range(POPULATION_SIZE):
        gene = create_gene(GENE_LENGTH)
        fitness, _ = calculate_fitness(gene)
        population[gene] = fitness

    return population

# ==========================================
# SELECTION (tournament)
# ==========================================

def tournament_selection(population, k=3):
    # pick k random individuals and return the best
    individuals = list(population.keys())
    contenders = np.random.choice(individuals, min(k, len(individuals)), replace=False)
    best = max(contenders, key=lambda g: population[g])
    return best


def selection(population):
    # select two distinct parents via tournament selection
    parent1 = tournament_selection(population)
    parent2 = tournament_selection(population)
    while parent2 == parent1:
        parent2 = tournament_selection(population)
    return parent1, parent2

# ==========================================
# CROSSOVER (two-point)
# ==========================================

def crossover(parent1, parent2):
    l = len(parent1)
    if l < 3:
        return parent1, parent2
    a = np.random.randint(1, l-1)
    b = np.random.randint(a, l)
    child1 = parent1[:a] + parent2[a:b] + parent1[b:]
    child2 = parent2[:a] + parent1[a:b] + parent2[b:]
    return child1, child2

# ==========================================
# MUTATION
# ==========================================

def mutation(gene):
    gene = list(gene)
    for i in range(len(gene)):
        if np.random.rand() < MUTATION_RATE:
            gene[i] = np.random.choice(MOVES)
    return ''.join(gene)


# ==========================================
# REPAIR OPERATOR
# ==========================================

def repair_gene(gene):
    # simple repair: whenever a step would cause collision, replace it
    position = START
    data = list(gene)
    for i, step in enumerate(data):
        next_position = move(position, step)
        if next_position in OBSTACLES or next_position == position:
            # choose move that reduces manhattan distance and not into obstacle
            candidates = MOVES.copy()
            best = None
            best_dist = float('inf')
            for m in candidates:
                p = move(position, m)
                if p in OBSTACLES:
                    continue
                d = abs(p[0]-GOAL[0]) + abs(p[1]-GOAL[1])
                if d < best_dist:
                    best_dist = d
                    best = m
            if best is not None:
                data[i] = best
                position = move(position, best)
            else:
                # pick random safe move
                safe = [m for m in MOVES if move(position, m) not in OBSTACLES and move(position, m) != position]
                if safe:
                    data[i] = np.random.choice(safe)
                    position = move(position, data[i])
                # else keep original (will be penalized)
        else:
            position = next_position
    return ''.join(data)

# ==========================================
# REGENERATION (elitism + fill)
# ==========================================

def regeneration(population, children, elitism=2):
    # keep elites
    sorted_pop = sorted(population.items(), key=lambda x: x[1], reverse=True)
    elites = dict(sorted_pop[:elitism])

    # create new population starting with elites
    new_pop = dict(elites)

    # fill rest with existing best until space for children
    candidates = [g for g, _ in sorted_pop]
    idx = 0
    while len(new_pop) < POPULATION_SIZE - len(children):
        g = candidates[idx]
        if g not in new_pop:
            new_pop[g] = population[g]
        idx += 1

    # add repaired children
    for child in children:
        repaired = repair_gene(child)
        fitness, _ = calculate_fitness(repaired)
        # ensure unique key (if duplicate, append small suffix)
        key = repaired
        if key in new_pop:
            key = repaired + str(np.random.randint(0, 9999))
        new_pop[key] = fitness

    # if still not full (unlikely), fill with random
    while len(new_pop) < POPULATION_SIZE:
        gene = create_gene(GENE_LENGTH)
        f, _ = calculate_fitness(gene)
        new_pop[gene] = f

    return new_pop

# ==========================================
# DISPLAY PATH
# ==========================================

def display_path(best_gene):

    position = START

    path_x = [position[0]]
    path_y = [position[1]]

    for step in best_gene:

        next_position = move(position, step)

        if next_position not in OBSTACLES:
            position = next_position

        path_x.append(position[0])
        path_y.append(position[1])

    # Plot environment
    plt.figure(figsize=(8, 8))

    # Obstacles
    for obs in OBSTACLES:
        plt.scatter(obs[0], obs[1], marker='s', s=300, c='black')

    # Start and Goal
    plt.scatter(START[0], START[1], s=300, c='green')
    plt.scatter(GOAL[0], GOAL[1], s=300, c='red')

    # Robot path
    plt.plot(path_x, path_y, linewidth=2, marker='o')

    plt.xlim(-1, GRID_SIZE)
    plt.ylim(-1, GRID_SIZE)

    plt.grid(True)
    plt.title("Genetic Algorithm Motion Planning")
    plt.show()


def get_path_positions(gene):
    position = START
    path_x = [position[0]]
    path_y = [position[1]]
    for step in gene:
        next_position = move(position, step)
        if next_position not in OBSTACLES:
            position = next_position
        path_x.append(position[0])
        path_y.append(position[1])
    return path_x, path_y

# ==========================================
# MAIN PROGRAM
# ==========================================
print("===================================")
print(" GENETIC ALGORITHM MOTION PLANNING ")
print("===================================")

start_time = datetime.datetime.now()

population = create_population()

# --- interactive plot setup ---
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
# draw obstacles, start, goal
for obs in OBSTACLES:
    ax.scatter(obs[0], obs[1], marker='s', s=300, c='black')
ax.scatter(START[0], START[1], s=300, c='green')
ax.scatter(GOAL[0], GOAL[1], s=300, c='red')
line, = ax.plot([], [], linewidth=2, marker='o')
ax.set_xlim(-1, GRID_SIZE)
ax.set_ylim(-1, GRID_SIZE)
ax.grid(True)


for generation in range(MAX_GENERATION):

    # Selection
    parent1, parent2 = selection(population)

    # Crossover
    child1, child2 = crossover(parent1, parent2)

    # Mutation
    child1 = mutation(child1)
    child2 = mutation(child2)

    # Regeneration
    population = regeneration(population, [child1, child2])

    # Best individual
    best_gene = max(population, key=population.get)
    best_fitness = population[best_gene]

    _, final_position = calculate_fitness(best_gene)

    # update interactive plot with current best path
    path_x, path_y = get_path_positions(best_gene)
    line.set_data(path_x, path_y)
    fig.canvas.draw()
    plt.pause(0.05)

    print(f"Generation {generation+1}")
    print(f"Best Gene     : {best_gene}")
    print(f"Fitness       : {best_fitness:.4f}")
    print(f"Final Position: {final_position}")
    print("-----------------------------------")

    # Stop if goal reached
    if final_position == GOAL:
        print("Goal Reached!")
        break

end_time = datetime.datetime.now()

print("Execution Time :", end_time - start_time)

# Display best path
plt.ioff()
# close interactive figure to avoid duplicate windows
plt.close(fig)
# final display to keep the window open
display_path(best_gene)
