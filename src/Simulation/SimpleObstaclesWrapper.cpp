//
// Created by nicco on 13/06/2023.
//

#include "Simulation/SimpleObstaclesWrapper.hpp"

SimpleObstaclesWrapper::SimpleObstaclesWrapper(const nlohmann::json &obstaclesJson) :
    AbstractObstaclesWrapper(getObstaclesFromJson(obstaclesJson))
{}

SimpleObstaclesWrapper::SimpleObstaclesWrapper(ObstaclesMap obstaclesMap) :
    AbstractObstaclesWrapper({std::move(obstaclesMap)})
{}

SpawnedObstaclesSet SimpleObstaclesWrapper::get() const{
    auto obstacles = savedObstacles | std::views::transform([](CompressedCoord pos) -> SpawnedObstacle{
        return{0, pos};
    });
    return {obstacles.begin(), obstacles.end()};
}

void SimpleObstaclesWrapper::update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) {
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
