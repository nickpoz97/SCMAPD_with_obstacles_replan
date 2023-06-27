//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SIMPLEOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_SIMPLEOBSTACLESWRAPPER_HPP

#include "Simulation/AbstractObstaclesWrapper.hpp"

class SimpleObstaclesWrapper : public AbstractObstaclesWrapper{
public:
    explicit SimpleObstaclesWrapper(const nlohmann::json &obstaclesJson);
    explicit SimpleObstaclesWrapper(ObstaclesMap obstaclesMap);

    void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) override;
    SpawnedObstaclesSet get() const override;
private:
    std::unordered_set<CompressedCoord> savedObstacles;
};


#endif //SIMULTANEOUS_CMAPD_SIMPLEOBSTACLESWRAPPER_HPP
