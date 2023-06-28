//
// Created by nicco on 17/06/2023.
//

#include "Simulation/SmartSimulator.hpp"
#include "SIPP.h"
#include "SingleAgentSolver.h"

SmartSimulator::SmartSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                               const nlohmann::json &obstaclesJson) :
        AbstractSimulator(std::move(runningAgents), std::move(ambientMap), obstaclesJson),
        predictor{obstaclesJson, computeSeed(runningAgents)}
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

std::unordered_map<int, bool> SmartSimulator::getBestChoices(const SpawnedObstaclesSet &visibleObstacles) const {
    std::unordered_map<int, bool> bestChoicesMap;

    for(const auto& ra : runningAgents){
        auto nextPos = ra.getNextPosition();
        auto raId = ra.getAgentId();

        if(visibleObstacles.contains({0, nextPos})){
            double waitPenalty = 0;
            double rePlanPenalty = 0;

            // take into account the obstacle
            auto noObsScore = computeNoObsScore(raId);

            // ignore the obstacle
            auto obsScore = computeObsScore(nextPos, raId);

            const auto& p = predictor.getIntervalProbabilities(nextPos);

            for(auto [interval, prob] : p){
                waitPenalty += prob * (noObsScore + interval);
                rePlanPenalty += prob * (obsScore);
            }

            bestChoicesMap[raId] = waitPenalty < rePlanPenalty;
        }
    }

    return bestChoicesMap;
}

bool SmartSimulator::newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const{
    const auto& gauss = predictor.getDistribution(pos);
    auto interval = actualSpawnTime - firstSpawnTime;

    return interval > gauss.mu && gauss.getProb(interval) <= 0.01;
}

std::unordered_set<CompressedCoord>
SmartSimulator::getNewObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t) {
    using RetType = decltype(SmartSimulator::getNewObstacles(obstaclesPositions, t));

    RetType newObstaclesPos;

    for(auto pos : obstaclesPositions){
        // update if necessary
        if(!foundObstacles.contains(pos) || newAppearance(pos, foundObstacles[pos], t)){
            foundObstacles[pos] = t;
            newObstaclesPos.insert(pos);
        }
    }

    return newObstaclesPos;
}


void SmartSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

//    auto visibleObstacles = obsWrapper->get();
//
//    auto bestChoices = getBestChoices(visibleObstacles);

//    for(const auto& [raId, wait] : bestChoices){
//
//    }
}
