//
// Created by nicco on 13/06/2023.
//

#include "Simulation/RePlanSimulator.hpp"
#include "Simulation/RePlanObstaclesWrapper.hpp"

void RePlanSimulator::doSimulationStep(TimeStep t) {
    auto nextPositions = getNextPositions();

    obsWrapper->update(t, nextPositions);

    auto predictedObstaclesSet{getPredictedObstacles(t)};

    auto obsAlongThePath = nextPositions |
        std::views::filter([&predictedObstaclesSet](CompressedCoord cc){return predictedObstaclesSet.contains({1, cc});});

    // check if there are predicted obstacles along the path
    if(!obsAlongThePath.empty()){
        auto pbsInstance = generatePBSInstance(predictedObstaclesSet, extractPBSCheckpoints());
        auto paths = solveWithPBS(pbsInstance);
        updatePlannedPaths(paths);
    }
}

SpawnedObstaclesSet RePlanSimulator::getPredictedObstacles(TimeStep actualT) const {
    SpawnedObstaclesSet predictedObstacles{};

    for(const auto& [obsT, obstacles] : obsWrapper->get()){
        std::ranges::for_each(obstacles, [obsT, actualT, &predictedObstacles](CompressedCoord cc){
            predictedObstacles.emplace(obsT - actualT, cc);
        });
    }
    return predictedObstacles;
}

RePlanSimulator::RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                                 const nlohmann::json &obstaclesJson) :
    AbstractSimulator{std::move(runningAgents), std::move(ambientMap)}
{
    obsWrapper = std::make_unique<RePlanObstaclesWrapper>(computeSeed(this->runningAgents), obstaclesJson);
}

RePlanSimulator::RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                                 ObstaclesMap obstaclesMap, ProbabilitiesMap probabilitiesMap) :
        AbstractSimulator{std::move(runningAgents), std::move(ambientMap)}
{
    obsWrapper = std::make_unique<RePlanObstaclesWrapper>(
        computeSeed(this->runningAgents),
        std::move(obstaclesMap),
        std::move(probabilitiesMap)
    );
}

Instance
RePlanSimulator::generatePBSInstance(const SpawnedObstaclesSet& sOSet, const vector<std::vector<CompressedCoord>>& checkpoints) const{
    return AbstractSimulator::generatePBSInstance({}, sOSet, checkpoints);
}
