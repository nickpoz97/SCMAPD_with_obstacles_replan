//
// Created by nicco on 13/06/2023.
//

#include "Simulation/WaitObstaclesWrapper.hpp"

WaitObstaclesWrapper::WaitObstaclesWrapper(const nlohmann::json &obstaclesJson) :
    AbstractObstaclesWrapper({}, getObstaclesFromJson(obstaclesJson))
{}

WaitObstaclesWrapper::WaitObstaclesWrapper(ObstaclesMap obstaclesMap) :
    AbstractObstaclesWrapper({}, {std::move(obstaclesMap)})
{}

ObstaclesMap WaitObstaclesWrapper::get() const{
    return ObstaclesMap{{1, savedObstacles}};
}

void WaitObstaclesWrapper::update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) {
    for(CompressedCoord cc : nextPositions){
        // obstacle is present and visible
        if(obstaclesMap[actualT].contains(cc)){
            savedObstacles.insert(cc);
        }
        // obstacle is not present (erase it if stored)
        else{
            savedObstacles.erase(cc);
        }
    }
}
