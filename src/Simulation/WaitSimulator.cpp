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

WaitSimulator::WaitSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                             ObstaclesMap obstaclesMap) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap)}
{
    obsWrapper = std::make_unique<WaitObstaclesWrapper>(std::move(obstaclesMap));
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
            setRePlan(nextPos);
        }
    }
    if(rePlan){
        auto nextPosRange = getWaitingAgentsIds() | std::views::transform([this](int aId){
            return std::make_pair(aId, runningAgents[aId].getNextPosition());
        });
        std::unordered_map nextPosMap{nextPosRange.begin(), nextPosRange.end()};

        // obstacles are considered like walls
        auto pbsInstance = generatePBSInstance(noCrossPositions, extractPBSCheckpoints(getWaitingAgentsIds()));
        updatePlannedPaths(solveWithPBS(pbsInstance));

        extendWaitingPositions(nextPosMap);
        rePlan = false;
    }
    else{
        extendWaitingPositions();
    }

    // obsAgentsMap.empty() <=> noCrossPositions.empty()
    assert((!obsAgentsMap.empty() || noCrossPositions.empty()) && (!noCrossPositions.empty() || obsAgentsMap.empty()));
}

void WaitSimulator::setRePlan(int formerObstaclePos) {
    assert(obsAgentsMap.contains(formerObstaclePos));

    // these positions are now free
    std::ranges::for_each(
        obsAgentsMap[formerObstaclePos],
        [this](int raId){noCrossPositions.erase(runningAgents[raId].getActualPosition());}
    );
    // erase waiting agents
    obsAgentsMap.erase(formerObstaclePos);
    // erase obstacle
    noCrossPositions.erase(formerObstaclePos);
    rePlan = true;
}

void WaitSimulator::wait(int waitingAgentIndex, int obstaclePos) {
    obsAgentsMap[obstaclePos].insert(waitingAgentIndex);
    noCrossPositions.insert(runningAgents[waitingAgentIndex].getActualPosition());
}

void WaitSimulator::extendWaitingPositions() {

    auto nextPosMap = getWaitingAgentsIds() | std::views::transform([this](int aId){
        return std::make_pair(aId, runningAgents[aId].getNextPosition());
    });

    extendWaitingPositions({nextPosMap.begin(), nextPosMap.end()});
}

void WaitSimulator::extendWaitingPositions(const std::unordered_map<int, CompressedCoord>& wAgentsNextPos) {
    std::ranges::for_each(
        getWaitingAgentsIds(),
        [this, &wAgentsNextPos](int aId) {
            auto& actualAgent = runningAgents[aId];
            auto actualPath = actualAgent.getPlannedPath();

            Path extendedPath{actualPath.front(), actualPath.front(), wAgentsNextPos.at(aId)};
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
