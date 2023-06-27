//
// Created by nicco on 13/06/2023.
//

#include "Simulation/RePlanSimulator.hpp"
#include "Simulation/PredictedObstaclesWrapper.hpp"

void RePlanSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

    obsWrapper->update(t, nextPositions);

    auto predictedObstaclesSet{obsWrapper->get()};

    auto obsAlongThePath = nextPositions |
        std::views::filter([&predictedObstaclesSet](CompressedCoord cc){return predictedObstaclesSet.contains({1, cc});});

    // check if there are predicted obstacles along the path
    if(!obsAlongThePath.empty()){
        auto pbsInstance = generatePBSInstance(predictedObstaclesSet, extractPBSCheckpoints());
        auto paths = solveWithPBS(pbsInstance);
        updatePlannedPaths(paths);
    }
}

RePlanSimulator::RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                                 const nlohmann::json &obstaclesJson) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap)}
{
    obsWrapper = std::make_unique<PredictedObstaclesWrapper>(computeSeed(this->runningAgents), obstaclesJson);
}

Instance
RePlanSimulator::generatePBSInstance(const SpawnedObstaclesSet& sOSet, const vector<std::vector<CompressedCoord>>& checkpoints) const{
    return AbstractSimulator::generatePBSInstance({}, sOSet, checkpoints);
}
