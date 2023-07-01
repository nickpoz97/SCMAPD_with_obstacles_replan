import os
import json
import sys
import matplotlib.pyplot as plt

folder_path = sys.argv[1]
stat = sys.argv[2]
obs_type = sys.argv[3]

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

# Create boxplot for specified obs_type
if obs_type in files_by_obs_type:
    fig, ax = plt.subplots()
    fig.suptitle(obs_type)
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
    plt.show()
else:
    print(f"No data found for obs_type '{obs_type}'")
