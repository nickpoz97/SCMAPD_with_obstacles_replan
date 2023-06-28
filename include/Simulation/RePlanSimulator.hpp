//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP

#include "AbstractSimulator.hpp"
#include "Predictor.hpp"

class RePlanSimulator : virtual public AbstractSimulator {
public:
    RePlanSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson);
protected:
    [[nodiscard]] Instance generatePBSInstance(const SpawnedObstaclesSet& sOSet, const vector<std::vector<CompressedCoord>>& checkpoints) const;
    Predictor predictor;

    void doSimulationStep(TimeStep t) override;
};


#endif //SIMULTANEOUS_CMAPD_REPLANSIMULATOR_HPP
