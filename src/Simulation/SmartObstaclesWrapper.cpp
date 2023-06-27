//
// Created by nicco on 17/06/2023.
//

#include "Simulation/SmartObstaclesWrapper.hpp"

SmartObstaclesWrapper::SmartObstaclesWrapper(const nlohmann::json &obstaclesJson) :
    AbstractPredictObstaclesWrapper{getProbabilitiesFromJson(obstaclesJson), getObstaclesFromJson(obstaclesJson)}
{}

bool SmartObstaclesWrapper::newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const{
    const auto& gauss = probabilitiesMap.at(pos);
    auto interval = actualSpawnTime - firstSpawnTime;

    return interval > gauss.mu && gauss.getProb(interval) <= 0.01;
}

std::unordered_set<CompressedCoord>
SmartObstaclesWrapper::updateFoundObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t) {
    using RetType = decltype(SmartObstaclesWrapper::updateFoundObstacles(obstaclesPositions,t));

    RetType newObstaclesPos;

    for(auto pos : obstaclesPositions){
        // update if necessary
        if(!foundObstacles.contains(pos) || newAppearance(pos, foundObstacles[pos], t)){
            foundObstacles[pos] = t;
            newObstaclesPos.insert(pos);
        }
    }

    return newObstaclesPos;
}

void SmartObstaclesWrapper::update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) {
    newObstacles.clear();

    auto visibleObstaclesPos =
        nextPositions |
        std::views::filter([&](CompressedCoord cc){
            return obstaclesMap.contains(actualT) && obstaclesMap[actualT].contains(cc);
        });

    newObstacles = updateFoundObstacles({visibleObstaclesPos.begin(), visibleObstaclesPos.end()}, actualT);
}

SpawnedObstaclesSet SmartObstaclesWrapper::get() const {
    auto obstacles = newObstacles | std::views::transform([](CompressedCoord pos) -> SpawnedObstacle{
        return{0, pos};
    });
    return {obstacles.begin(), obstacles.end()};
}
