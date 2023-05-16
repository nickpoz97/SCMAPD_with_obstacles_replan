import os
import argparse
import subprocess
from itertools import product

def generate_instances_dict(instances_dir):
    # 0, 1, ...
    instances_ids = {os.path.splitext(path)[0] for path in os.listdir(instances_dir)}

    return {id : (os.path.join(instances_dir, f"{id}.agents"), os.path.join(instances_dir, f"{id}.tasks")) for id in instances_ids}

def execute_tests(instances_dir, exe_path, grid_path, dm_path, out_dir_path):
    heuristics = {"MCA" : "MCA", "RMCAr" : "RMCA_R"}
    n_selected_tasks = [1, 3, 5]
    methods = {"RT" : "RANDOM_TASKS", "WT" : "WORST_TASKS"}

    instances_dict = generate_instances_dict(instances_dir)

    for n, kv_method, kv_heur in product(n_selected_tasks, methods.items(), heuristics.items()):
        for id, (agents_file_path, tasks_file_path) in instances_dict.items():

            output_json_name = f"{id}_{kv_heur[0]}_{kv_method[0]}_{n}.json"

            # already computed
            if output_json_name in os.listdir(out_dir_path):
                continue

            output_json_path = os.path.join(out_dir_path, output_json_name)

            command = [
                exe_path,
                '--dm', dm_path,
                '--m', grid_path,
                '--a', agents_file_path,
                '--t', tasks_file_path,
                '--h', kv_heur[1],
                '--obj', "TTD",
                '--metric', "DELAY",
                '--cutoff', "60",
                '--nt', str(n),
                '--mtd', kv_method[1],
                '--agents_info',
                '--out_path', output_json_path
            ]

            print(*command, sep=' ')

            subprocess.run(command, capture_output=False, check=False)


if __name__== "__main__":
    parser = argparse.ArgumentParser(description="Execute test on instances from specified directory to specified result directory")
    parser.add_argument("instances_dir", type=str, help="directory where instances are stored")
    parser.add_argument("exe_path", type=str, help="simultaneous cmapd exe path")
    parser.add_argument("dm_path", type=str, help="distance matrix path")
    parser.add_argument("grid_path", type=str, help="grid_path")
    parser.add_argument("out_dir_path", type=str, help="output directory path")
    args = parser.parse_args()

    instances_dir = os.path.abspath(args.instances_dir)
    exe_path = os.path.abspath(args.exe_path)
    grid_path = os.path.abspath(args.grid_path)
    dm_path = os.path.abspath(args.dm_path)
    out_dir_path = os.path.abspath(args.out_dir_path)

    execute_tests(instances_dir, exe_path, grid_path, dm_path, out_dir_path)
