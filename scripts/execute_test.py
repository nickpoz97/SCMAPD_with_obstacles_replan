import os
import argparse
import subprocess

def execute_test(instances_dir, exe_path, grid_path, dm_path):
    heuristics = ["MCA", "RMCA_A", "RMCA_R"]

    h_dirs = dict()
    for h in heuristics:
        res_dir = os.path.join(instances_dir, h)
        os.makedirs(res_dir, exist_ok=True)
        h_dirs[h] = res_dir

    # organize instances in a dict
    instances = dict()
    for filename in os.listdir(instances_dir):
        file_path = os.path.join(instances_dir, filename)
        if os.path.isfile(file_path):
            file_info = filename.split('.')
            index, extension = int(file_info[0]), file_info[1]
            if index not in instances:
                instances[index] = dict()
            instances[index][extension] = file_path

    for index, files in instances.items():
        a_ext = 'agents'
        t_ext = 'tasks'
        assert(a_ext in files and t_ext in files)

        for h in heuristics:
            result = subprocess.run(
                f"{exe_path} --m {grid_path} --dm {dm_path} --a {files[a_ext]} --t {files[t_ext]} --h {h}",
                shell=True,
                capture_output=True
            )

            assert(result.returncode == 0)
            paths_filename = str(index) + '.paths'
            stats_filename = str(index) + '.stats'

            rows = [row for row in result.stdout.decode('UTF-8').split('\n')]

            # including header row and first one
            boundary = int(rows[0]) + 2

            paths = rows[:boundary][1:]
            stats = rows[boundary:]

            with open(os.path.join(h_dirs[h], paths_filename), 'w') as paths_file:
                paths_file.writelines(paths)
            with open(os.path.join(h_dirs[h], stats_filename), 'w') as stats_file:
                stats_file.writelines(stats)


if __name__== "__main__":
    parser = argparse.ArgumentParser(description="Execute test on instances from specified directory")
    parser.add_argument("instances_dir", type=str, help="directory where instances are stored")
    parser.add_argument("exe_path", type=str, help="simultaneous cmapd exe path")
    parser.add_argument("dm_path", type=str, help="distance matrix path")
    parser.add_argument("grid_path", type=str, help="grid_path")
    args = parser.parse_args()

    instances_dir = os.path.normpath(args.instances_dir)
    exe_path = os.path.normpath(args.exe_path)
    grid_path = os.path.normpath(args.grid_path)
    dm_path = os.path.normpath(args.dm_path)

    execute_test(instances_dir, exe_path, grid_path, dm_path)
