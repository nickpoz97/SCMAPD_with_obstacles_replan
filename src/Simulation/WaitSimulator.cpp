//
// Created by nicco on 13/06/2023.
//

#include "Simulation/WaitSimulator.hpp"

#include <memory>

WaitSimulator::WaitSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap), obstaclesJson}
{}

void WaitSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();
    const auto visibleObstacles = obsWrapper.get(nextPositions, t);

    chooseStatusForAgents(nextPositions, visibleObstacles);

    if(needRePlan){
        rePlanFreeAgents();
    }
}

void WaitSimulator::wait(int waitingAgentIndex, int obstaclePos) {
    obsAgentsMap[obstaclePos].insert(waitingAgentIndex);
    runningAgents[waitingAgentIndex].forceWait();
}

void WaitSimulator::setRePlan(int formerObstaclePos) {
    assert(obsAgentsMap.contains(formerObstaclePos));

    // these positions are now free
    obsAgentsMap.erase(formerObstaclePos);

    needRePlan = true;
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

void WaitSimulator::chooseStatusForAgents(const std::vector<CompressedCoord> &nextPositions,
                                          const std::unordered_set<CompressedCoord> &visibleObstacles) {// forbidden places

    for(int i = 0 ; i < nextPositions.size() ; ++i){
        auto nextPos = nextPositions[i];
        // obstacle present -> agent becomes an obstacle and will need replanning
        if(getExtendedObstacles(visibleObstacles).contains(nextPos)){
            wait(i, nextPos);
        }
        // obstacle de spawned
        else if(obsAgentsMap.contains(nextPos)){
            setRePlan(nextPos);
        }
    }
}

void WaitSimulator::rePlanFreeAgents() {
    auto waitingAgentsIds = getWaitingAgentsIds();

    auto pbsInstance = generatePBSInstance({}, extractPBSCheckpoints(waitingAgentsIds));
    updatePlannedPaths(solveWithPBS(pbsInstance, waitingAgentsIds));

    needRePlan = false;
}

std::unordered_set<CompressedCoord>
WaitSimulator::getExtendedObstacles(const std::unordered_set<CompressedCoord> &visibleObstacles) const {
    auto extendedObstacles = visibleObstacles;

    std::ranges::copy(getWaitingAgentsPositions(), std::inserter(extendedObstacles, extendedObstacles.end()));
    return extendedObstacles;
}

std::unordered_set<CompressedCoord>
WaitSimulator::getWaitingAgentsPositions() const{
    std::unordered_set<CompressedCoord> waitingAgentsPositions{};

    std::ranges::for_each(
        obsAgentsMap | std::views::values,
        [&, this](const auto& waitingAgentIds){
            std::ranges::transform(
                waitingAgentIds,
                std::inserter(waitingAgentsPositions, waitingAgentsPositions.end()),
                [this](int waId){return runningAgents[waId].getActualPosition();}
            );
        }
    );

    return waitingAgentsPositions;
}