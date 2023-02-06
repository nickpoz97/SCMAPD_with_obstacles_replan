import argparse
import random
import os

parser = argparse.ArgumentParser(description="Generate random instances")

parser.add_argument("grid", metavar="grid_path", type=str, help="path of grid file")
parser.add_argument("a", metavar="A", type=int, help="number of agents")
parser.add_argument("t", metavar="T", type=int, help="number of tasks")
parser.add_argument("n", metavar="N", type=int, help="number of instances")
parser.add_argument("out", metavar="output_dir_path", type=str, help="instances output location")

args = parser.parse_args()

grid_path = args.grid
n_agents = args.a
n_tasks = args.t
n_instances = args.n

# extract possible positions for tasks and agents
candidates = list()

assert(os.path.isfile(grid_path))
with open(grid_path, "r") as file:
    lines = file.readlines()

    global grid
    grid = [['.'] * len(lines[0]) for i in range(len(lines))]

    for rowi, l in enumerate(lines): 
        for coli, symbol in enumerate(l.strip()):
            if symbol == 'G':
                candidates.append((rowi, coli))
            if symbol == '@': 
                grid[rowi][coli] = '@'

#print(grid)

# generate and print instances
assert(n_agents + 2 * n_tasks <= len(candidates))

for i in range(n_instances):
    instance_grid = grid.copy()

    random.shuffle(candidates)
    stringify_coord = lambda coord: str(coord[0]) + ',' + str(coord[1])

    agents_coords = candidates[:n_agents]
    tasks_coords = candidates[n_agents:n_agents + 2 * n_tasks]

    agents = ['\n' + stringify_coord(coord) for coord in agents_coords]

    get_sep = lambda index : '\n' if index % 2 == 0 else ','
    tasks = [get_sep(j) + stringify_coord(coord) for j, coord in enumerate(tasks_coords)]

    agents_path = str(i) + '.agents'
    tasks_path = str(i) + '.tasks'
    instance_grid_path = str(i) + '.grid'

    with open(os.path.join(args.out, agents_path), "w") as af:
        af.write(str(n_agents))
        af.writelines(agents)
        af.write('\n')

    with open(os.path.join(args.out, tasks_path), "w") as tf:
        tf.write(str(n_tasks))
        tf.writelines(tasks)
        tf.write('\n')

    for row, col in agents_coords:
        instance_grid[row][col] = 'a'
    for j, (row, col) in enumerate(tasks_coords):
        instance_grid[row][col] = 's' if j % 2 == 0 else 'g'

    grid_string = ''
    for j in range(len(grid)):
        grid_string += ''.join(grid[j]) + '\n'
        
    with open(os.path.join(args.out, instance_grid_path), "w") as igf:
        igf.write(grid_string)