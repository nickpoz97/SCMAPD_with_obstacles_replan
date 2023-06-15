//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP

#include "AbstractSimulator.hpp"

class RePlanSimulator : public AbstractSimulator {
public:
    RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson);

private:
    void doSimulationStep(TimeStep t) override;

    SpawnedObstaclesSet getPredictedObstacles(TimeStep actualT) const;
};


#endif //SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP
