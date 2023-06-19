//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP

#include "AbstractSimulator.hpp"

class RePlanSimulator : public AbstractSimulator {
public:
    RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson);

    RePlanSimulator(vector<RunningAgent> runningAgents, AmbientMap ambientMap, ObstaclesMap obstaclesMap,
                    ProbabilitiesMap probabilitiesMap);


private:
    [[nodiscard]] Instance generatePBSInstance(const SpawnedObstaclesSet& sOSet, const vector<std::vector<CompressedCoord>>& checkpoints) const;

    void doSimulationStep(TimeStep t) override;

    [[nodiscard]] SpawnedObstaclesSet getPredictedObstacles(TimeStep actualT) const;
};


#endif //SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP
