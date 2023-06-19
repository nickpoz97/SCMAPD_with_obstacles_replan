//
// Created by nicco on 17/06/2023.
//

#include "Simulation/SmartSimulator.hpp"
#include "Simulation/SmartObstaclesWrapper.hpp"
#include "SIPP.h"
#include "SingleAgentSolver.h"

SmartSimulator::SmartSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                               const nlohmann::json &obstaclesJson) :
        AbstractSimulator(std::move(runningAgents), std::move(ambientMap))
{
    obsWrapper = std::make_unique<SmartObstaclesWrapper>(obstaclesJson);
}

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
        {agentCPExtractor(ra, false)},
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        {}
    };

    auto h = SIPP{waitInstance, ra.getActualPosition()}.my_heuristic;

    assert(!ra.getPlannedCheckpoints().empty());

    // heuristic between actual pos and first checkpoint
    return h[ra.getActualPosition()][ra.getPlannedCheckpoints().front()];
}

std::unordered_map<int, bool> SmartSimulator::getBestChoices(const std::unordered_set<CompressedCoord> &visibleObstacles) const {
    std::unordered_map<int, bool> bestChoicesMap;

    for(const auto& ra : runningAgents){
        auto nextPos = ra.getNextPosition();
        auto raId = ra.getAgentId();

        if(visibleObstacles.contains(nextPos)){
            double waitPenalty = 0;
            double rePlanPenalty = 0;

            // take into account the obstacle
            auto noObsScore = computeNoObsScore(raId);

            // ignore the obstacle
            auto obsScore = computeObsScore(nextPos, raId);

            const auto& p = obsWrapper->getProbabilities(nextPos);

            for(auto [interval, prob] : p){
                waitPenalty += prob * (noObsScore + interval);
                rePlanPenalty += prob * (obsScore);
            }

            bestChoicesMap[raId] = waitPenalty < rePlanPenalty;
        }
    }

    return bestChoicesMap;
}
