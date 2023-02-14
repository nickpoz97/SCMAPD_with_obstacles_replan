import os
import argparse
import matplotlib.pyplot as plt
import itertools

import numpy as np
import math

heuristics = ['MCA', 'RMCA_R', 'RMCA_A']
stats = ['time', 'makespan', 'total travel time', 'total travel delay']

def load_data_dir_paths(instances_root: str, n_agents: int, n_tasks: int):
    data_dir_path = os.path.join(instances_root, str(n_agents) + '_' + str(n_tasks))

    path_maker = lambda name: os.path.join(data_dir_path, name) 
    return [path_maker(name) for name in os.listdir(data_dir_path) if os.path.isdir(path_maker(name))]

def extract_stats(stats_file_path: str):
    with open(stats_file_path, "r") as stats_file:
        return {id.strip(':').lower(): val for id, val in [line.strip().split('\t') for line in stats_file.readlines()]}

def load_stats_files_paths(data_dir_path: str):
    return [os.path.join(data_dir_path, name) for name in os.listdir(data_dir_path) if name.split('.')[-1] == "stats"]

stats_files_filter = lambda file_path: os.path.splitext(file_path)[-1] == '.stats'

def plot_instances(dir_paths: list, info_to_plot: str, heuristic: str, ax: plt.Axes):
    # filter heuristic choice
    dir_paths = [x for x in dir_paths if x.split(os.sep)[-1].find(heuristic, 0, len(heuristic)) != -1]

    for mode_dir in dir_paths:
        tree = mode_dir.split(os.sep)
        #print(tree)
        agents, tasks = (int(x) for x in tree[-2].split('_'))
        mode = tree[-1]
        #print(agents, tasks, mode)

        converter = int 
        if info_to_plot.lower() == "time":
            converter = float
        if info_to_plot.lower() == 'conflicts':
            converter = bool

        stats_files = [os.path.join(mode_dir, fname) for fname in filter(stats_files_filter ,os.listdir(mode_dir))]
        y_values = [converter(extract_stats(f)[info_to_plot.replace(' ', '_')]) for f in stats_files]
        x_values = range(len(y_values))

        #print(y_values)
        ax.plot(x_values, y_values, "o-", label = mode.replace(heuristic + '_', ''))
        ax.legend(loc="upper right")
        ax.set_title(f"{heuristic}")

def compute_stat_mean(stat_name: str, stat_dir_path: str):
    assert(stat_name in stats)

    stats_files = [filename for filename in os.listdir(stat_dir_path) if os.path.splitext(filename)[-1] == '.stats']
    values = list()

    converter = int 
    if stat_name.lower() == "time":
        converter = float
    if stat_name.lower() == 'conflicts':
        converter = bool

    for sf in stats_files:
        values.append(converter(extract_stats(os.path.join(stat_dir_path, sf))[stat_name.replace(' ', '_')]))
    return np.mean(values)

def comparison_plot(instances_root: str):
    modes = ['eager', 'forward_only', 'lazy']

    agents_tasks_dirs = [os.path.join(instances_root, at_dir) for at_dir in os.listdir(instances_root) if os.path.isdir(os.path.join(instances_root, at_dir))]

    data_dict = dict()

    max_vals = {stat : 0 for stat in stats}
    min_vals = {stat: math.inf for stat in stats}

    for h, mode, info_to_plot in itertools.product(heuristics, modes, stats):
        subfolder_id = h + '_' + mode

        x = data_dict[(info_to_plot, h, mode, 'x')] = list()
        y = data_dict[(info_to_plot, h, mode, 'y')] = list()
        z = data_dict[(info_to_plot, h, mode, 'z')] = list()

        for a_t_dir in agents_tasks_dirs:
            a, t = map(int, a_t_dir.split(os.sep)[-1].split('_'))
            x.append(a)
            y.append(t)
            stat_mean_val = compute_stat_mean(info_to_plot, os.path.join(a_t_dir, subfolder_id))

            max_vals[info_to_plot] = max(max_vals[info_to_plot], stat_mean_val)
            min_vals[info_to_plot] = min(min_vals[info_to_plot], stat_mean_val)
            z.append(stat_mean_val)

    for h, mode, info_to_plot in itertools.product(heuristics, modes, stats):
        x = data_dict[(info_to_plot, h, mode, 'x')]
        y = data_dict[(info_to_plot, h, mode, 'y')]
        z = data_dict[(info_to_plot, h, mode, 'z')]
        
        fig = plt.figure()
        plt.scatter(x, y, c=z, vmin=min_vals[info_to_plot], vmax=max_vals[info_to_plot], s=400, marker="s")
        plt.colorbar(label="value")
        plt.title(f"{h} {mode}: {info_to_plot}")

        plt.xticks(np.arange(np.min(x), np.max(x)+1, (np.max(x) - np.min(x)) / (len(set(x))-1) ))
        plt.yticks(np.arange(np.min(y), np.max(y)+1, (np.max(y) - np.min(y)) / (len(set(y))-1) ))
        plt.xlabel("agents")
        plt.ylabel("tasks")
        fig.tight_layout()
        plt.savefig(os.path.join(instances_root, f"comprehensive_{info_to_plot}_{h}_{mode}.png"))
        plt.close()

        

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Execute test on grouped instances from specified directory")
    parser.add_argument("instances_root", type=str, help="directory where directories of instances are stored")
    args = parser.parse_args()
    instances_root = args.instances_root

    comparison_plot(instances_root)

    for subdir in [dir for dir in os.listdir(instances_root) if os.path.isdir(os.path.join(instances_root, dir))]:
        a, t = map(lambda v: int(v), subdir.split('_'))
        for h, info_to_plot in itertools.product(heuristics, stats):
            out_dir_path = os.path.join(instances_root, subdir)
            assert(os.path.exists(out_dir_path))

            fig, axs = plt.subplots(nrows=1, ncols=len(heuristics), sharey=True)
            fig.set_size_inches(11,5)

            for i, h in enumerate(heuristics):
                plt.suptitle(f"{info_to_plot}, n_agents: {a}, n_tasks: {t}")
                dir_paths = load_data_dir_paths("instances", a, t)
                plot_instances(dir_paths, info_to_plot, h, axs[i])
            fig.tight_layout()

            filepath = os.path.join(out_dir_path, info_to_plot.replace(' ', '_') + ".png")
            print(f"saving {filepath}")
            plt.savefig(filepath)
            plt.close()
