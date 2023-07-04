import os
import json
import sys
import matplotlib.pyplot as plt
from itertools import product

stats = ["makespan", "total_travel_time", "total_travel_distance"]
obs_types = ["obs_long", "obs_short", "obs_smart"]

folder_path = sys.argv[1]
target_dir = os.path.normpath(sys.argv[2])

for (stat, obs_type) in product(stats, obs_types):
    # Group files by obs_type
    files_by_obs_type = {}
    for file_name in os.listdir(folder_path):
        if file_name.endswith('.json'):
            file_parts = file_name.split('-')
            if len(file_parts) == 3 and file_parts[2].endswith('.json'):
                file_obs_type = file_parts[1]
                if file_obs_type == obs_type:
                    if obs_type not in files_by_obs_type:
                        files_by_obs_type[obs_type] = []
                    files_by_obs_type[obs_type].append(file_name)

    obs_map = {
        "obs_long": "Long interval obstacles",
        "obs_short": "Short interval obstacles",
        "obs_smart": "Variable interval obstacles"
    }

    # Create boxplot for specified obs_type
    if obs_type in files_by_obs_type:
        fig, ax = plt.subplots()
        fig.suptitle(f"{obs_map[obs_type]}: {stat.replace('_', ' ')}")
        data_by_strategy = {}
        for file_name in files_by_obs_type[obs_type]:
            strategy = file_name.split('-')[2].split('.')[0]
            if strategy not in data_by_strategy:
                data_by_strategy[strategy] = []
            with open(os.path.join(folder_path, file_name)) as f:
                file_data = json.load(f)
                if stat in file_data['stats']:
                    data_by_strategy[strategy].append(file_data['stats'][stat])
        boxplot_data = []
        strategies = []
        for strategy in data_by_strategy.keys():
            if len(data_by_strategy[strategy]) > 0:
                boxplot_data.append(data_by_strategy[strategy])
                strategies.append(strategy)
            else:
                boxplot_data.append([])
        ax.boxplot(boxplot_data)
        ax.set_xticklabels(strategies)
        plt.savefig(os.path.join(target_dir, f'{obs_type}-{stat}.png'))
    else:
        print(f"No data found for obs_type '{obs_type}'")
