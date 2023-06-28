//
// Created by nicco on 13/06/2023.
//

#include "Simulation/RePlanSimulator.hpp"

void RePlanSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

    auto visibleObstacles{obsWrapper.get(nextPositions, t)};

    // check if there are predicted obstacles along the path
    if(!visibleObstacles.empty()){
        auto pbsInstance = generatePBSInstance(predictor.predictWithMemory(visibleObstacles, t), extractPBSCheckpoints());
        auto paths = solveWithPBS(pbsInstance);
        updatePlannedPaths(paths);
    }
}

RePlanSimulator::RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                                 const nlohmann::json &obstaclesJson) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap), obstaclesJson},
    predictor{obstaclesJson, computeSeed(runningAgents)}
{}

Instance
RePlanSimulator::generatePBSInstance(const SpawnedObstaclesSet& sOSet, const vector<std::vector<CompressedCoord>>& checkpoints) const{
    return AbstractSimulator::generatePBSInstance({}, sOSet, checkpoints);
}
