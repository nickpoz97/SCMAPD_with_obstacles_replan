import argparse
import os
import sys
import shutil

# find other script
sys.path.append(os.path.realpath(__file__))

from generate_instances import *

parser = argparse.ArgumentParser(description="Generate multiple random instances")

parser.add_argument("grid", metavar="grid_path", type=str, help="path of grid file")

parser.add_argument("min_a", type=int, help="minimum number of agents")
parser.add_argument("max_a", type=int, help="maximum number of agents")
parser.add_argument("step_a", type=int, help="step between minimum and maximum n of agents")

parser.add_argument("min_t", type=int, help="minimum number of tasks")
parser.add_argument("max_t", type=int, help="maximum number of tasks")
parser.add_argument("step_t", type=int, help="step between minimum and maximum n of tasks")

parser.add_argument("n", type=int, help="number of instances per agent_task couple")

parser.add_argument("out", metavar="output_dir_path", type=str, help="instances output location")
args = parser.parse_args()

for a in range(args.min_a, args.max_a + 1, args.step_a):
    for t in range(args.min_t, args.max_t + 1, args.step_t):
        out_dir = os.path.join(args.out, str(a) + '_' + str(t))
        os.makedirs(out_dir)
        try:
            gen_instances(args.grid, a, t, args.n, out_dir)        
        except NoMorePositions as ex:
            print(ex)
            shutil.rmtree(out_dir)
