import os
import argparse

def load_data_dir_paths(instances_root: str, n_agents: int, n_tasks: int):
    data_dir_path = os.path.join(instances_root, str(n_agents) + '_' + str(n_tasks))

    path_maker = lambda name: os.path.join(data_dir_path, name) 
    return [path_maker(name) for name in os.listdir(data_dir_path) if os.path.isdir(path_maker(name))]

def extract_stats(stats_file_path: str):
    with open(stats_file_path, "r") as stats_file:
        return {id: val for id, val in [line.strip().split('\t') for line in stats_file.readlines()]}

def load_stats_files_paths(data_dir_path: str):
    return [os.path.join(data_dir_path, name) for name in os.listdir(data_dir_path) if name.split('.')[-1] == "stats"]

parser = argparse.ArgumentParser(description="Execute test on grouped instances from specified directory")
parser.add_argument("instances_root", type=str, help="directory where directories of instances are stored")
args = parser.parse_args()

dir_paths = load_data_dir_paths("instances", 40, 40)
print(dir_paths)
stat_file = load_stats_files_paths(dir_paths[0])[0]
print(stat_file)
print(extract_stats(stat_file))
