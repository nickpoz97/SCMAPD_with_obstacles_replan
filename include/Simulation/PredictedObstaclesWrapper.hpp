//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_PREDICTEDOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_PREDICTEDOBSTACLESWRAPPER_HPP

#include "Simulation/AbstractPredictObstaclesWrapper.hpp"

class PredictedObstaclesWrapper : public AbstractPredictObstaclesWrapper{
public:
    PredictedObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson);

    PredictedObstaclesWrapper(size_t seed, ObstaclesMap obstaclesMap, ProbabilitiesMap probabilitiesMap);

    void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) override;
    SpawnedObstaclesSet get() const override;
private:
    ObstaclesMap predictedObstacles;
    std::default_random_engine gen;

    TimeStep actualTimeStep = 0;
};


#endif //SIMULTANEOUS_CMAPD_PREDICTEDOBSTACLESWRAPPER_HPP
