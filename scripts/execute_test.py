import os
import argparse
import subprocess

parser = argparse.ArgumentParser(description="Execute test on instances from specified directory")
parser.add_argument("instances_dir", type=str, help="directory where instances are stored")
parser.add_argument("exe_path", type=str, help="simultaneous cmapd exe path")
parser.add_argument("dm_path", type=str, help="distance matrix path")
parser.add_argument("grid_path", type=str, help="grid_path")
parser.add_argument("h", type=str, help="heuristic choice", choices=["MCA", "RMCA_A", "RMCA_R"])
args = parser.parse_args()

instances_dir = os.path.normpath(args.instances_dir)
exe_path = os.path.normpath(args.exe_path)
grid_path = os.path.normpath(args.grid_path)
dm_path = os.path.normpath(args.dm_path)

instances = dict()
for filename in os.listdir(args.instances_dir):
    file_info = filename.split('.')
    index, extension = int(file_info[0]), file_info[1]
    if index not in instances:
        instances[index] = dict()
    instances[index][extension] = os.path.join(instances_dir, filename)

for index, files in instances.items():
    a_ext = 'agents'
    t_ext = 'tasks'
    assert(a_ext in files and t_ext in files)

    heur = args.h
    result = subprocess.run(
        f"{exe_path} --m {grid_path} --dm {dm_path} --a {files[a_ext]} --t {files[t_ext]} --h {heur}",
        shell=True,
        capture_output=True
    )
    assert(result.returncode == 0)
    path_filename = str(index) + '_' + heur + '.paths'
    stats_filename = str(index) + '.stats'

    rows = [row for row in result.stdout.decode('UTF-8').split('\n')]

    # including header row and first one
    boundary = int(rows[0]) + 2

    paths = rows[:boundary][1:]
    stats = rows[boundary:]

    with open(os.path.join(instances_dir, path_filename), 'w') as paths_file:
        paths_file.writelines(paths)
    with open(os.path.join(instances_dir, stats_filename), 'w') as stats_file:
        stats_file.writelines(stats)
