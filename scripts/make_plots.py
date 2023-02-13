import os
import argparse
import matplotlib.pyplot as plt
import itertools

def load_data_dir_paths(instances_root: str, n_agents: int, n_tasks: int):
    data_dir_path = os.path.join(instances_root, str(n_agents) + '_' + str(n_tasks))

    path_maker = lambda name: os.path.join(data_dir_path, name) 
    return [path_maker(name) for name in os.listdir(data_dir_path) if os.path.isdir(path_maker(name))]

def extract_stats(stats_file_path: str):
    with open(stats_file_path, "r") as stats_file:
        return {id.strip(':').lower(): val for id, val in [line.strip().split('\t') for line in stats_file.readlines()]}

def load_stats_files_paths(data_dir_path: str):
    return [os.path.join(data_dir_path, name) for name in os.listdir(data_dir_path) if name.split('.')[-1] == "stats"]

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

        files_filter = lambda fp: os.path.splitext(fp)[-1] == '.stats'
        stats_files = [os.path.join(mode_dir, fname) for fname in filter(files_filter ,os.listdir(mode_dir))]
        y_values = [converter(extract_stats(f)[info_to_plot.replace(' ', '_')]) for f in stats_files]
        x_values = range(len(y_values))

        #print(y_values)
        ax.plot(x_values, y_values, "o-", label = mode.replace(heuristic + '_', ''))
        ax.legend(loc="upper right")
        ax.set_title(f"{heuristic}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Execute test on grouped instances from specified directory")
    parser.add_argument("instances_root", type=str, help="directory where directories of instances are stored")
    args = parser.parse_args()
    instances_root = args.instances_root

    heuristics = ['MCA', 'RMCA_R', 'RMCA_A']
    stats = ['time', 'makespan', 'total travel time', 'total travel delay']

    for subdir in os.listdir(instances_root):
        a, t = map(lambda v: int(v), subdir.split('_'))
        for h, info_to_plot in itertools.product(heuristics, stats):
            out_dir_path = os.path.join(instances_root, subdir)
            assert(os.path.exists(out_dir_path))

            fig, axs = plt.subplots(nrows=1, ncols=len(heuristics), sharey=True)
            fig.set_size_inches(11,5)

            a = 40
            t = 130
            for i, h in enumerate(heuristics):
                plt.suptitle(f"{info_to_plot}, n_agents: {a}, n_tasks: {t}")
                dir_paths = load_data_dir_paths("instances", 40, 130)
                plot_instances(dir_paths, info_to_plot, h, axs[i])
            fig.tight_layout()

            filepath = os.path.join(out_dir_path, info_to_plot.replace(' ', '_') + ".png")
            print(f"saving {filepath}")
            plt.savefig(filepath)
            plt.close()

    exit()
