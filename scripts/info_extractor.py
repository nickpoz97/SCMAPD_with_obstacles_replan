import subprocess
import os
import json

def execute_instance(exe_path: str, agents_file_path: str, tasks_file_path: str, heur: str, obj: str, metric: str, nt: int, method: str, cutoff: int):
    command = [
        exe_path,
        '--a', agents_file_path,
        '--t', tasks_file_path,
        '--h', heur,
        '--obj', obj,
        '--metric', metric,
        '--cutoff', str(cutoff),
        '--nt', str(nt),
        '--mtd', method
    ]
    print(*command, sep=' ')

    result = subprocess.run(command, capture_output=True, check=True)
    return result.stdout.decode("UTF-8")

def instance_stats(decoded_output: str):
    data = json.loads(decoded_output)
    return data["stats"]

def all_stats(exe_path: str, instances_root: str, settings: list):
    tasks_files = [os.path.join(instances_root, name) for name in filter(lambda fname: os.path.splitext(fname)[-1] == '.tasks', os.listdir(instances_root))]
    agents_files = [os.path.join(instances_root, name) for name in filter(lambda fname: os.path.splitext(fname)[-1] == '.agents', os.listdir(instances_root))]
    
    return [instance_stats(execute_instance(exe_path, af, tf, *settings)) for af, tf in zip(agents_files, tasks_files)]

if __name__ == "__main__":
    print(all_stats("./scmapd", "../instances/40_130", ["MCA", "MAKESPAN", "DELAY", 5, "WORST_TASKS", 0]))
