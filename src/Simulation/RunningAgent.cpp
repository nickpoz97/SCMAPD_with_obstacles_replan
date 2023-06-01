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
        Path plannedPath{};

        std::ranges::transform(
            agentsStats["path"],
            std::back_inserter(plannedPath),
            [&dm](const nlohmann::json& jsonPath){return dm.from2Dto1D(jsonPath[0], jsonPath[1]);}
        );

        // load checkpoints
        CheckPoints plannedCheckpoints;

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

RunningAgent::RunningAgent(int agentId, Path plannedPath, CheckPoints plannedCheckpoints)
        : agentId(agentId), plannedPath(std::move(plannedPath)), plannedCheckpoints(std::move(plannedCheckpoints)) {}

CompressedCoord RunningAgent::getActualPosition() const {
    assert(!plannedPath.empty());
    return plannedPath.front();
}

void RunningAgent::stepAndUpdate(){
    assert(!plannedPath.empty() && !plannedCheckpoints.empty());

    // do a step
    if(plannedPath.size() > 1) {
        plannedPath.erase(plannedPath.begin());

        if(plannedCheckpoints.size() > 1 && plannedPath.front() == plannedCheckpoints.front()){
            plannedCheckpoints.pop_front();
        }
    }

    // only one position -> last waypoint reached
    assert(
        !(plannedPath.size() == 1) ||
            (plannedPath.front() == plannedCheckpoints.front() && plannedCheckpoints.size() == 1)
    );
}

const Path &RunningAgent::getPlannedPath() const {
    return plannedPath;
}

void RunningAgent::setPlannedPath(Path newPlannedPath) {
    RunningAgent::plannedPath = std::move(newPlannedPath);
}

int RunningAgent::getAgentId() const {
    return agentId;
}

const CheckPoints &RunningAgent::getPlannedCheckpoints() const {
    return plannedCheckpoints;
}

bool RunningAgent::hasFinished() const {
    assert(!plannedPath.empty());
    return plannedPath.size() == 1;
}

std::optional<CompressedCoord> RunningAgent::getNextPosition() const {
    assert(plannedPath.size() >= 1);
    return plannedPath.size() >= 2 ? std::optional{plannedPath[1]} : std::nullopt;
}

std::size_t hash_value(const RunningAgent& s){
    size_t seed = 0;

    auto hashCombiner = [&seed](CompressedCoord cc) {boost::hash_combine(seed, cc);};

    std::ranges::for_each(s.getPlannedPath(), hashCombiner);
    //std::ranges::for_each(s.getPlannedCheckpoints(), hashCombiner);

    return seed;
}
