from itertools import product
import random
import os

grid_path = os.path.normpath('data/grid.txt')
#agents_path = os.path.normpath('instances/20_500_paper/0.agents')

#subdir_name, file_name = agents_path.split(os.sep)[-2:]

#instance_id = file_name.split('.')[0]

# saving obstacles here
obstacles_dir = 'obstacles'
if not os.path.exists(obstacles_dir):
    os.makedirs(obstacles_dir)

# saving obstacles probs here
probs_dir = 'probs'
if not os.path.exists(probs_dir):
    os.makedirs(probs_dir)

occupation_types = {'quick': (6, 2), 'med': (10, 3), 'slow': (50, 15)}
configs = {
    'quick-only' : (1.0, 0.0, 0.0),
    'med-only' : (0.0, 1.0, 0.0), 
    'slow-only' : (0.0, 0.0, 1.0), 
    'all-balanced' : (0.33, 0.34, 0.33),
    'quick-skewed' : (0.6, 0.2, 0.2),
    'slow-skewed' : (0.2, 0.2, 0.6)
}
n_types = len(occupation_types)
n_obstacles_list = [n_types, n_types + 2, n_types + 4]

spawn_time_limit = 50

# get possible obstacle positions and number of rows and cols
candidates = set()

with open(grid_path, 'r') as grid_file:
    n_rows, n_cols = 0, 0

    rows = grid_file.readlines()
    n_rows, n_cols = len(rows), len(rows[0].strip())

    for rowi, row in enumerate(rows):
        for coli, cell in enumerate(row.strip()):
            if cell == '.' or cell == 'G':
                candidates.add(rowi * n_cols + coli)

# # get agents positions
# agents_pos = set()
# with open(agents_path, 'r') as agents_file:
#     n_agents = int(agents_file.readline().strip())
#     for val in agents_file.readlines():
#         row_str, col_str = val.strip().split(',')
#         agents_pos.add((int(row_str), int(col_str)))
#     assert(n_agents == len(agents_pos))

# # obstacles cannot spawn on agents
# candidates.difference_update(agents_pos)

candidates = list(candidates)

quick_positions = list()
med_positions = list()
slow_positions = list()

obstacles = list()

for c, n in product(configs.items(), n_obstacles_list):
    random.shuffle(candidates)

    c_name, c_probs = c

    quick_threshold = len(candidates) * c_probs[0]
    med_threshold = quick_threshold + len(candidates) * c_probs[1]

    i = 0

    candidates_with_probs = list()
    while i < quick_threshold:
        candidates_with_probs.append((candidates[i], occupation_types['quick']))
        i += 1
    while i < med_threshold:
        candidates_with_probs.append((candidates[i], occupation_types['med']))
        i += 1
    while i < len(candidates):
        candidates_with_probs.append((candidates[i], occupation_types['slow']))
        i += 1


    chosen_candidates = random.sample(candidates_with_probs, n)
    obstacles = [(pos, random.randint(1, spawn_time_limit), round(random.gauss(*prob))) for pos, prob in chosen_candidates]

    upper_bound = spawn_time_limit + occupation_types['slow'][0] + occupation_types['slow'][1] * 4

    obstacles_to_print = [[-1 for _ in range(n)] for _ in range(upper_bound)]
    for i, (pos, spawn_time, duration) in enumerate(obstacles):
        for t in range(spawn_time, spawn_time + duration):
            obstacles_to_print[t][i] = pos

    while obstacles_to_print[-1] == [-1 for _ in range(n)]:
        obstacles_to_print.pop()

    with open(os.path.join(obstacles_dir, '_'.join([c_name, str(n)]) + '.csv'), 'w') as out_file:
        print(*['obs_' + str(i) for i in range(n)], sep=',', end='\n', file=out_file)
        
        for row in obstacles_to_print:
            print(*row, sep=',', end='\n', file=out_file)

    with open(os.path.join(probs_dir, '_'.join([c_name, str(n)]) + '.csv'), 'w') as out_file:
        print(*['pos', 'mu', 'std'], sep=',', end='\n', file=out_file)

        for pos, (mu, std) in candidates_with_probs:
            print(*[pos, mu, std], sep=',', end='\n', file=out_file)
