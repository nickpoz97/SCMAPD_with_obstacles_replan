//
// Created by nicco on 13/06/2023.
//

#include "Simulation/ObstaclesWrapper.hpp"

ObstaclesWrapper::ObstaclesWrapper(const nlohmann::json &obstaclesJson) :
    obstaclesMap{getObstaclesFromJson(obstaclesJson)}
{}

ObstaclesMap ObstaclesWrapper::getObstaclesFromJson(const nlohmann::json &obstaclesJson) {
    ObstaclesMap obstacles{};

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        TimeStep from = obsObj["t"];
        TimeStep to = from + static_cast<TimeStep>(obsObj["interval"]);

        for(auto t = from ; t < to ; ++t){
            obstacles[t].emplace(static_cast<CompressedCoord>(obsObj["pos"]));
        }
    }

    return obstacles;
}

std::unordered_set<CompressedCoord>
ObstaclesWrapper::get(const std::vector<CompressedCoord> &nextPositions, TimeStep actualT) const {
    if(!obstaclesMap.contains(actualT)){
        return {};
    }

    auto visibleObstacles =
        nextPositions |
        std::views::filter([this, actualT](CompressedCoord np){return obstaclesMap.at(actualT).contains(np);});

    return {visibleObstacles.begin(), visibleObstacles.end()};
}
