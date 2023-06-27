//
// Created by nicco on 13/06/2023.
//

#include "Simulation/AbstractObstaclesWrapper.hpp"

AbstractObstaclesWrapper::AbstractObstaclesWrapper(ObstaclesMap obstaclesMap) :
    obstaclesMap{std::move(obstaclesMap)}
{}

ObstaclesMap AbstractObstaclesWrapper::getObstaclesFromJson(const nlohmann::json &obstaclesJson) {
    ObstaclesMap obstacles{};

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        TimeStep from = obsObj["t"];
        TimeStep to = from + static_cast<TimeStep>(obsObj["interval"]);

        for(auto t = from ; t < to ; ++t){
            obstacles[t].insert(static_cast<CompressedCoord>(obsObj["pos"]));
        }
    }

    return obstacles;
}
