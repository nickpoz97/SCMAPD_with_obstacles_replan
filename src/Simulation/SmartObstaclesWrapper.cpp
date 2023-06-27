//
// Created by nicco on 17/06/2023.
//

#include "Simulation/SmartObstaclesWrapper.hpp"

SmartObstaclesWrapper::SmartObstaclesWrapper(const nlohmann::json &obstaclesJson) :
    AbstractObstaclesWrapper{getProbabilitiesFromJson(obstaclesJson), getObstaclesFromJson(obstaclesJson)}
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
    SpawnedObstaclesSet sOSet;

    // todo implement this
//    for(const auto& kv : newObstacles){
//        auto relativeT = kv.first - newObstacles;
//
//        for(const auto& v : kv.second){
//            sOSet.emplace(relativeT, v);
//        }
//    }

    return sOSet;
}
