//
// Created by nicco on 13/06/2023.
//

#include "Simulation/WaitObstaclesWrapper.hpp"

WaitObstaclesWrapper::WaitObstaclesWrapper(const nlohmann::json &obstaclesJson) :
    AbstractObstaclesWrapper(obstaclesJson)
{}

ObstaclesMap WaitObstaclesWrapper::get() const{
    return ObstaclesMap{{1, visibleObstacles}};
}

void WaitObstaclesWrapper::update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) {
    for(CompressedCoord cc : nextPositions){
        // obstacle is present and visible
        if(obstaclesMap[actualT].contains(cc)){
            visibleObstacles.insert(cc);
        }
        // obstacle is not present (erase it if stored)
        else{
            visibleObstacles.erase(cc);
        }
    }
}
