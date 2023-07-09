import os
import json
import sys
import matplotlib.pyplot as plt
from itertools import product

obs_types = ["obs_long", "obs_short", "obs_smart"]

folder_path = sys.argv[1]
target_dir = os.path.normpath(sys.argv[2])
stat = sys.argv[3]

fig, axs = plt.subplots(1, 3, figsize=(12, 4), sharey=True)

ymin = float('inf')
ymax = float('-inf')

for i, obs_type in enumerate(obs_types):
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
        "obs_smart": "Balanced interval obstacles"
    }

    # Create boxplot for specified obs_type
    if obs_type in files_by_obs_type:
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
        axs[i].boxplot(boxplot_data)
        axs[i].set_xticks(range(1, len(strategies) + 1))
        axs[i].set_xticklabels(strategies)
        axs[i].set_title(obs_map[obs_type])

        # Update ymin and ymax values
        ymin = min(ymin, min([min(data) for data in boxplot_data if len(data) > 0]))
        ymax = max(ymax, max([max(data) for data in boxplot_data if len(data) > 0]))
    else:
        print(f"No data found for obs_type '{obs_type}'")

# Set y-axis range to be the same for all three boxplots
padding = (ymax - ymin) * 0.1
for ax in axs:
    ax.set_ylim(ymin - padding, ymax + padding)

plt.savefig(os.path.join(target_dir, f'{stat}.png'))
