import os
import json
import argparse
from random import gauss, shuffle

def generate_obstacles(base_path):
    for filename in os.listdir(base_path):
        if filename.endswith("_base.json"):
            d = int(filename.split("_")[0])
            with open(os.path.join(base_path, filename), "r") as f:
                data = json.load(f)
                agents = data["agents"]
                obstacles = []
                probability = []
                for agent in agents:
                    path = agent["path"]
                    for i in range(1, len(path) - 1):
                        if path[i][1] == 17:
                            row = path[i][0]
                            if row in [2, 6, 10, 14, 18]:
                                pos = row * 35 + 17
                                if not any(o["pos"] == pos for o in obstacles):
                                    t = i - 1
                                    obstacles.append({"t": t, "pos": pos})
                                    probability.append({"pos": pos})
                obs_path = os.path.join(base_path, f"{d}_obs")
                os.makedirs(obs_path, exist_ok=True)
                for obs_type in ["long", "short", "smart"]:
                    obs_data = {"obstacles": obstacles.copy(), "probability": probability.copy()}
                    if obs_type == "long":
                        mu, sigma = 50, 5
                        for o in obs_data["obstacles"]:
                            o["interval"] = int(gauss(mu, sigma))
                        for p in obs_data["probability"]:
                            p["mu"] = mu
                            p["sigma"] = sigma
                    elif obs_type == "short":
                        mu, sigma = 4, 1
                        for o in obs_data["obstacles"]:
                            o["interval"] = int(gauss(mu, sigma))
                        for p in obs_data["probability"]:
                            p["mu"] = mu
                            p["sigma"] = sigma
                    else:
                        half = len(obs_data["obstacles"]) // 2
                        shuffle(obs_data["obstacles"])
                        for i, o in enumerate(obs_data["obstacles"]):
                            if i < half:
                                mu, sigma = 50, 5
                            else:
                                mu, sigma = 4, 1
                            o["interval"] = int(gauss(mu, sigma))
                            p = next(p for p in obs_data["probability"] if p["pos"] == o["pos"])
                            p["mu"] = mu
                            p["sigma"] = sigma
                    with open(os.path.join(obs_path, f"obs_{obs_type}.json"), "w") as f:
                        json.dump(obs_data, f)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("base_path", help="Path to the folder containing the d_base.json files")
    args = parser.parse_args()
    generate_obstacles(args.base_path)
