import os
import re
import sys
import subprocess

if len(sys.argv) < 3:
    print('too few aguments')
    sys.exit(1)

input_folder_path = os.path.abspath(sys.argv[1])
exe_path = os.path.abspath(sys.argv[2])

base_pattern = re.compile(r'(\d+)_base\.json')
obs_folder_pattern = re.compile(r'(\d+)_obs')

base_indices = []
folder_indices = []

for filename in os.listdir(input_folder_path):
    base_match = base_pattern.match(filename)
    obs_folder_match = obs_folder_pattern.match(filename)
    if base_match:
        base_indices.append(int(base_match.group(1)))
    elif obs_folder_match:
        folder_indices.append(int(obs_folder_match.group(1)))

indices = set(base_indices) & set(folder_indices)

for i in sorted(indices):
    base_file_path = os.path.join(input_folder_path, f'{i}_base.json')

    obs_dir = os.path.join(input_folder_path, f'{i}_obs')
    obs_files_paths = [os.path.join(obs_dir, obs_file) for obs_file in os.listdir(obs_dir)]

    for obs_file_path in obs_files_paths:
        obs_file_type = base=os.path.splitext(os.path.basename(obs_file_path))[0]

        for strategy in ['RE_PLAN', 'WAIT', 'SMART']:
            command = [
                exe_path,
                '--plans', base_file_path,
                '--obstacles', obs_file_path,
                '--strategy', strategy,
                '--out', os.path.join(input_folder_path, f'{i}-{obs_file_type}-{strategy.lower()}.json')
            ]
            print(*command, sep=' ')

            subprocess.run(command, check=True)