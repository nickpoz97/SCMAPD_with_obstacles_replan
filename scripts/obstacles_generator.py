from itertools import product
import random

grid_path = 'grid_file.txt'
agents_path = 'agents_file.agents'

occupation_types = {'quick': (4, 2), 'med': (10, 3), 'slow': (50, 15)}
configs = {
    'quick_only' : (1.0, 0.0, 0.0),
    'med_only' : (0.0, 1.0, 0.0), 
    'slow_only' : (0.0, 0.0, 1.0), 
    'all_balanced' : (0.33, 0.34, 0.33),
    'quick_skewed' : (0.6, 0.2, 0.2),
    'slow_skewed' : (0.2, 0.2, 0.6)
}
n_types = len(occupation_types)
n_obstacles_list = [n_types, n_types + 2, n_types + 4]

spawn_time_limit = 50

# get possible obstacle positions and number of rows and cols
candidates = set()
n_rows, n_cols = 0, 0
with open(grid_path, 'r') as grid_file:
    rows = grid_file.readlines()
    n_rows, n_cols = len(rows), len(rows[0].strip())

    for rowi, row in enumerate(rows):
        for coli, cell in enumerate(row.strip()):
            if cell == '.' or cell == 'G':
                candidates.add((rowi, coli))

# get agents positions
agents_pos = list()
with open(agents_path, 'r') as agents_file:
    n_agents = int(agents_file.readline().strip())
    for val in agents_file.readlines():
        val_int = int(val.strip())
        agents_pos.append((val_int // n_cols, val_int % n_cols))
    assert(n_agents == len(agents_pos))

# obstacles cannot spawn on agents
candidates.difference_update(agents_pos)

# obtain shuffled list
candidates = list(candidates)
random.shuffle(candidates)

quick_positions = list()
med_positions = list()
slow_positions = list()

obstacles = list()

for c, n in product(configs.items(), n_obstacles_list):
    c_name, c_probs = c

    quick_threshold = len(candidates) * c_probs[0]
    med_threshold = quick_threshold + len(candidates) * c_probs[1]

    i = 0
    while i < quick_threshold:
        candidates[i] = (candidates[i], occupation_types['quick'])
        i += 1
    while i < med_threshold:
        candidates[i] = (candidates[i], occupation_types['med'])
        i += 1
    while i < len(candidates):
        candidates[i] = (candidates[i], occupation_types['slow'])
        i += 1


    chosen_candidates = random.sample(candidates, n)
    obstacles = [(pos, random.randint(1, spawn_time_limit), random.gauss(*prob)) for pos, prob in chosen_candidates]
        
