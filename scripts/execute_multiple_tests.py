import argparse
import os
import sys

# find other script
sys.path.append(os.path.realpath(__file__))

from execute_test import *

parser = argparse.ArgumentParser(description="Execute test on grouped instances from specified directory")
parser.add_argument("instances_root", type=str, help="directory where directories of instances are stored")
parser.add_argument("exe_path", type=str, help="simultaneous cmapd exe path")
parser.add_argument("dm_path", type=str, help="distance matrix path")
parser.add_argument("grid_path", type=str, help="grid_path")
args = parser.parse_args()

instances_root = os.path.normpath(args.instances_root)
for subdir in os.listdir(instances_root):
    print(subdir + " instances")
    execute_test(
        os.path.join(instances_root, subdir),
        os.path.normpath(args.exe_path),
        os.path.normpath(args.grid_path),
        os.path.normpath(args.dm_path)
    )
