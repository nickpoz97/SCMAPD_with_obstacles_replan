//
// Created by nicco on 13/06/2023.
//

#include "Simulation/WaitSimulator.hpp"

#include <memory>
#include "Simulation/WaitObstaclesWrapper.hpp"

WaitSimulator::WaitSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap)}
{
    obsWrapper = std::make_unique<WaitObstaclesWrapper>(obstaclesJson);
}

void WaitSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

    // update the internal state
    obsWrapper->update(t, nextPositions);

    // only obstacles present in the next time step are interesting
    const auto actualObstacles = (obsWrapper->get()).at(1);

    // forbidden places
    for(CompressedCoord pos : actualObstacles){
        noCrossPositions.insert(pos);
    }

    for(int i = 0 ; i < nextPositions.size() ; ++i){
        auto nextPos = nextPositions[i];
        // obstacle present -> agent becomes an obstacle and will need replanning
        if(actualObstacles.contains(nextPos)){
            wait(i, nextPos);
        }
        // obstacle de spawned
        else if(obsAgentsMap.contains(nextPos)){
            rePlan(i, nextPos);
        }
    }
    extendWaitingPositions();

    // obsAgentsMap.empty() <=> noCrossPositions.empty()
    assert((!obsAgentsMap.empty() || noCrossPositions.empty()) && (!noCrossPositions.empty() || obsAgentsMap.empty()));
}

void WaitSimulator::rePlan(int freeAgentId, int formerObstaclePos) {// these positions are now free
    std::erase_if(noCrossPositions, [this, formerObstaclePos](CompressedCoord cc){
        return obsAgentsMap[formerObstaclePos].contains(cc);}
    );
    // erase waiting agents
    obsAgentsMap[formerObstaclePos].erase(freeAgentId);

    // obstacles are considered like walls
    auto pbsInstance = generatePBSInstance(noCrossPositions, extractPBSCheckpoints(getWaitingAgentsIds()));
    updatePlannedPaths(solveWithPBS(pbsInstance));
}

void WaitSimulator::wait(int waitingAgentIndex, int obstaclePos) {
    obsAgentsMap[obstaclePos].insert(waitingAgentIndex);
    noCrossPositions.insert(runningAgents[waitingAgentIndex].getActualPosition());
}

void WaitSimulator::extendWaitingPositions() {// extend with waiting pos
    std::ranges::for_each(
            getWaitingAgentsIds() | std::views::transform([this](int aId){
            return std::make_pair(aId, runningAgents[aId].getNextPosition());
        }),
            [this](const auto& kv) {
            auto [aId, nextPos] = kv;
            auto& actualAgent = runningAgents[aId];
            auto actualPath = actualAgent.getPlannedPath();

            Path extendedPath{actualPath.front(), actualPath.front(), nextPos};
            actualAgent.setPlannedPath(extendedPath);
        }
    );
}

std::unordered_set<int> WaitSimulator::getWaitingAgentsIds() const {
    decltype(WaitSimulator::getWaitingAgentsIds()) waitingAgentsIds;

    std::ranges::for_each(
        obsAgentsMap | std::views::values,
        [&waitingAgentsIds](const auto& agentIds){
            std::ranges::copy(agentIds, std::inserter(waitingAgentsIds, waitingAgentsIds.end()));
        }
    );

    return waitingAgentsIds;
}

