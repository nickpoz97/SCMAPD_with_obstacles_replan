//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_WAITOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_WAITOBSTACLESWRAPPER_HPP

#include "Simulation/AbstractObstaclesWrapper.hpp"

class WaitObstaclesWrapper : public AbstractObstaclesWrapper{
public:
    explicit WaitObstaclesWrapper(const nlohmann::json &obstaclesJson);
    void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) override;
    ObstaclesMap get() const override;
private:
    std::unordered_set<CompressedCoord> savedObstacles;
};


#endif //SIMULTANEOUS_CMAPD_WAITOBSTACLESWRAPPER_HPP
