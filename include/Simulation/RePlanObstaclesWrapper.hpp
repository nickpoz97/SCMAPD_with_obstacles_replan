//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_REPLANOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_REPLANOBSTACLESWRAPPER_HPP

#include "Simulation/AbstractObstaclesWrapper.hpp"

class RePlanObstaclesWrapper : public AbstractObstaclesWrapper{
public:
    explicit RePlanObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson);
    void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) override;
    ObstaclesMap get() const override;
private:
    ObstaclesMap predictedObstacles;
    std::default_random_engine gen;
};


#endif //SIMULTANEOUS_CMAPD_REPLANOBSTACLESWRAPPER_HPP
