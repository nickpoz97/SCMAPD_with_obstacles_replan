import random

def gen_overlapping_instances(candidates:list[tuple[int, int]], n_agents:int, n_tasks:int):
    agents_locs = random.sample(candidates, n_agents)
    candidates = [x for x in candidates if x not in set(agents_locs)]

    tasks_locs = [tuple(*random.sample(candidates, 1)) for i in range(2 * n_tasks)]
    return agents_locs, tasks_locs

def gen_paper_overlapping_instances(robot_candidates:list[tuple[int,int]], tasks_candidates:list[tuple[int,int]], n_agents:int, n_tasks:int):
    agents_locs = random.sample(robot_candidates, n_agents)
    tasks_locs = [tuple(*random.sample(tasks_candidates, 1)) for i in range(2 * n_tasks)]
    return agents_locs, tasks_locs