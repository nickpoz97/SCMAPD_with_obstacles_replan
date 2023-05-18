//
// Created by nicco on 18/05/2023.
//

#include "Simulation/RunningAgent.hpp"

std::vector<RunningAgent> loadPlansFromJson(const nlohmann::json &j, const DistanceMatrix &dm) {
    std::vector<RunningAgent> runningAgents{};
    const auto& agentsJson{ j["agents"] };

    runningAgents.reserve(agentsJson.size());

    // iterate over all agents
    for(const auto& agentsStats : agentsJson){
        // load path
        PlannedPath plannedPath{};

        std::ranges::transform(
            agentsStats["path"],
            std::back_inserter(plannedPath),
            [&dm](const nlohmann::json& jsonPath){return dm.from2Dto1D(jsonPath[0], jsonPath[1]);}
        );

        // load checkpoints
        PlannedCheckpoints plannedCheckpoints;

        std::ranges::transform(
            agentsStats["waypoints"],
            std::back_inserter(plannedCheckpoints),
            [&dm](const nlohmann::json& jsonCheckpoint){
                const auto& coord = jsonCheckpoint["coords"];
                return dm.from2Dto1D(coord[0], coord[1]);
            }
        );

        runningAgents.emplace_back(agentsStats["index"], std::move(plannedPath), std::move(plannedCheckpoints));
    }

    return runningAgents;
}

RunningAgent::RunningAgent(int agentId, PlannedPath plannedPath, PlannedCheckpoints plannedCheckpoints)
        : agentId(agentId), plannedPath(std::move(plannedPath)), plannedCheckpoints(std::move(plannedCheckpoints)) {}
