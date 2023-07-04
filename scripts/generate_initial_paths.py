from info_extractor import execute_instance

import os
import re
import sys

if len(sys.argv) < 4:
    print('Usage: python script.py /path/to/folder')
    sys.exit(1)

input_folder_path = os.path.abspath(sys.argv[1])
output_folder_path = os.path.abspath(sys.argv[2])
exe_path = os.path.abspath(sys.argv[3])

agents_pattern = re.compile(r'(\d+)\.agents')
tasks_pattern = re.compile(r'(\d+)\.tasks')

agents_indices = []
tasks_indices = []

for filename in os.listdir(input_folder_path):
    agents_match = agents_pattern.match(filename)
    tasks_match = tasks_pattern.match(filename)
    if agents_match:
        agents_indices.append(int(agents_match.group(1)))
    elif tasks_match:
        tasks_indices.append(int(tasks_match.group(1)))

indices = set(agents_indices) & set(tasks_indices)

for i in sorted(indices):
        agents_file_path = os.path.join(input_folder_path, f'{i}.agents')
        tasks_file_path = os.path.join(input_folder_path, f'{i}.tasks')
        result_path = os.path.join(output_folder_path, f'{i}_base.json')
        
        # process the data
        execute_instance(exe_path, agents_file_path, tasks_file_path, 'MCA', 'TTD', 'DELAY', 1, 'RANDOM_TASKS', 2, result_path)
