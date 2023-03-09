import subprocess
import os

def execute_instance(exe_path: str, agents_file_path: str, tasks_file_path: str, heur: str, obj: str, metric: str, nt: int, method: str):
    command = [
        exe_path,
        '--a', agents_file_path,
        '--t', tasks_file_path,
        '--h', heur,
        '--obj', obj,
        '--metric', metric,
        '--cutoff', nt,
        '--mtd', method
    ]

    result = subprocess.run(command, capture_output=True, check=True)
    return result.stdout.decode("UTF-8")

def instance_stats(decoded_output: str):
    lines = decoded_output.strip().split('\n')
    header_line_n = 3

    infos = dict()

    splitter = lambda s: s.split(':\t')
    infos["stats"] = {
        splitter(l)[0]: splitter(l)[1] for l in lines[:header_line_n]
    }
    infos["agents_span"] = [
        int(l.split('\t')[1])-1 for l in lines[header_line_n+1:]
    ]
    return infos

def all_stats(exe_path: str, instances_root: str):
    

    # tasks_files = [os.path.join(instances_root, name) for name in filter(lambda fname: os.path.splitext(fname)[-1] == '.tasks', os.listdir(instances_root))]
    # agents_files = [os.path.join(instances_root, name) for name in filter(lambda fname: os.path.splitext(fname)[-1] == '.agents', os.listdir(instances_root))]
    
    # return [instance_stats(execute_instance(exe_path, af, tf)) for af, tf in zip(agents_files, tasks_files)]
