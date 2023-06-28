//
// Created by nicco on 17/06/2023.
//

#include "Simulation/SmartSimulator.hpp"
#include "SIPP.h"
#include "SingleAgentSolver.h"

SmartSimulator::SmartSimulator(const std::vector<RunningAgent>& runningAgents, const AmbientMap& ambientMap,
                               const nlohmann::json &obstaclesJson) :
        AbstractSimulator(runningAgents, ambientMap, obstaclesJson),
        WaitSimulator(runningAgents, ambientMap, obstaclesJson),
        RePlanSimulator(runningAgents, ambientMap, obstaclesJson)
{}

int SmartSimulator::computeNoObsScore(int raId) const {
    return getScore(raId, ambientMap.getGrid());
}

int SmartSimulator::computeObsScore(CompressedCoord obsPos, int raId) const {
    auto modGrid = ambientMap.getGrid();
    // obstacle fixed
    modGrid[obsPos] = true;

    return getScore(raId, modGrid);
}

int SmartSimulator::getScore(int raId, const vector<bool> &grid) const {
    const auto& ra = runningAgents[raId];

    Instance waitInstance{
        grid,
        {getExtendedCheckpoints(ra)},
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        {}
    };

    auto h = SIPP{waitInstance, ra.getActualPosition()}.my_heuristic;

    assert(!ra.getPlannedCheckpoints().empty());

    // heuristic between actual pos and first checkpoint
    return h[ra.getActualPosition()][ra.getPlannedCheckpoints().front()];
}

void SmartSimulator::applySmartChoice(const std::unordered_set<CompressedCoord> &allVisibleObstacles, TimeStep t) {
    const auto newVisibleObstacles = updateAndGetNewObstacles(allVisibleObstacles, t);

    for(const auto& raId : getWaitingAgentsIds()){
        auto nextPos = runningAgents[raId].getNextPosition();

        // old obstacle already handled
        if(!newVisibleObstacles.contains(nextPos)){
            continue;
        }

        double waitPenalty = 0;
        double rePlanPenalty = 0;

        // take into account the obstacle
        auto noObsScore = computeNoObsScore(raId);

        // ignore the obstacle
        auto obsScore = computeObsScore(nextPos, raId);

        const auto& p = predictor.getIntervalProbabilities(nextPos);

        // compute penalties
        for(auto [interval, prob] : p){
            waitPenalty += prob * (noObsScore + interval);
            rePlanPenalty += prob * (obsScore);
        }

        // remove from wait if replan is better
        if(rePlanPenalty < waitPenalty){
            needRePlan = true;

            if(obsAgentsMap.contains(nextPos)){
                auto& nextPosAgentsMap = obsAgentsMap.at(nextPos);
                nextPosAgentsMap.erase(raId);
                if(nextPosAgentsMap.empty()){
                    obsAgentsMap.erase(nextPos);
                }
            }
        }
    }
}

bool SmartSimulator::newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const{
    auto newObsOldPos = [=, this](){
        const auto& gauss = predictor.getDistribution(pos);
        auto interval = actualSpawnTime - firstSpawnTime;

        return interval > gauss.mu && gauss.getProb(interval) <= 0.01;
    };

    return !foundObstacles.contains(pos) || newObsOldPos();
}

std::unordered_set<CompressedCoord>
SmartSimulator::updateAndGetNewObstacles(const std::unordered_set<CompressedCoord> &obstaclesPositions, TimeStep t) {
    using RetType = decltype(SmartSimulator::updateAndGetNewObstacles(obstaclesPositions, t));

    RetType newObstaclesPos;

    for(auto pos : obstaclesPositions){
        // update if necessary
        if(newAppearance(pos, foundObstacles[pos], t)){
            foundObstacles[pos] = t;
            newObstaclesPos.insert(pos);
        }
    }

    return newObstaclesPos;
}

void SmartSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

    const auto allVisibleObstacles = obsWrapper.get(nextPositions, t);

    // all in wait (replan only if obstacle disappeared)
    chooseStatusForAgents(nextPositions, allVisibleObstacles);

    extendWaitingPositions();

    applySmartChoice(allVisibleObstacles, t);

    if(needRePlan){
        applyRePlan(allVisibleObstacles);
    }

//    auto visibleObstacles = obsWrapper->get();
//
//    auto bestChoices = getBestChoices(visibleObstacles);

//    for(const auto& [raId, wait] : bestChoices){
//
//    }
}

void SmartSimulator::applyRePlan(const std::unordered_set<CompressedCoord> &visibleObstacles) {
    auto waitingAgentsIds = getWaitingAgentsIds();

    const auto pbsInstance = RePlanSimulator::generatePBSInstance(
        predictor.predict(visibleObstacles),
        extractPBSCheckpoints(waitingAgentsIds)
    );

    updatePlannedPaths(solveWithPBS(pbsInstance, waitingAgentsIds));
}
