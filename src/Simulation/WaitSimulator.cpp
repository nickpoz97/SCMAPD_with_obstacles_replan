//
// Created by nicco on 13/06/2023.
//

#include "Simulation/WaitSimulator.hpp"

#include <memory>
#include "Simulation/SimpleObstaclesWrapper.hpp"

WaitSimulator::WaitSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap)}
{
    obsWrapper = std::make_unique<SimpleObstaclesWrapper>(obstaclesJson);
}

void WaitSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

    // update the internal state
    obsWrapper->update(t, nextPositions);

    // only obstacles present in the next time step are interesting
    const auto actualObstacles = obsWrapper->get();


    chooseStatusForAgents(nextPositions, actualObstacles);

    if(needRePlan){
        rePlanFreeAgents();
    }
    else{
        extendWaitingPositions();
    }

    // obsAgentsMap.empty() <=> noCrossPositions.empty()
    assert((!obsAgentsMap.empty() || noCrossPositions.empty()) && (!noCrossPositions.empty() || obsAgentsMap.empty()));
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
    needRePlan = true;
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

Instance WaitSimulator::generatePBSInstance(const std::unordered_set<CompressedCoord> &fixedObstacles,
                                                    const vector<std::vector<CompressedCoord>> &checkPoints) const {
    return AbstractSimulator::generatePBSInstance(fixedObstacles, {}, checkPoints);
}

void WaitSimulator::chooseStatusForAgents(const vector<CompressedCoord> &nextPositions,
                                          const SpawnedObstaclesSet &visibleObstacles) {// forbidden places
    for(const auto& obs : visibleObstacles){
        noCrossPositions.insert(obs.position);
    }

    for(int i = 0 ; i < nextPositions.size() ; ++i){
        auto nextPos = nextPositions[i];
        // obstacle present -> agent becomes an obstacle and will need replanning
        if(visibleObstacles.contains({1, nextPos})){
            wait(i, nextPos);
        }
            // obstacle de spawned
        else if(obsAgentsMap.contains(nextPos)){
            setRePlan(nextPos);
        }
    }
}

void WaitSimulator::rePlanFreeAgents() {
    auto nextPosRange = getWaitingAgentsIds() | std::views::transform([this](int aId){
        return std::make_pair(aId, runningAgents[aId].getNextPosition());
    });
    std::unordered_map nextPosMap{nextPosRange.begin(), nextPosRange.end()};

    // obstacles are considered like walls
    auto pbsInstance = generatePBSInstance(noCrossPositions, extractPBSCheckpoints(getWaitingAgentsIds()));
    updatePlannedPaths(solveWithPBS(pbsInstance));

    extendWaitingPositions(nextPosMap);
    needRePlan = false;
}
