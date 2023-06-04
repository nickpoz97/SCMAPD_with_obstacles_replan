//
// Created by nicco on 04/06/2023.
//

#include "Simulation/ObstaclesWrapper.hpp"

SpawnedObstaclesSet ObstaclesWrapper::updateAndGet(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) {
    // no more interesting ones removed
    std::erase_if(predictedObstacles, [actualT](const auto& kv){return kv.first < actualT;});
    std::erase_if(trueObstacles, [actualT](const auto& kv){return kv.first < actualT;});

    // obtain obstacles found by agents and get their predicted persistence
    auto obstaclesWithPermanence = nextPositions |
       std::views::filter([&](CompressedCoord cc){return trueObstacles[actualT].contains(cc);}) |
       std::views::transform([&](CompressedCoord cc) -> ObstaclePersistence{
           auto [mu, std] = probabilitiesMap[cc];
           std::normal_distribution<float> d(static_cast<float>(mu), static_cast<float>(std));
           return {cc, static_cast<Interval>(d(gen))};}
    );

    // update section
    for(const auto& owp : obstaclesWithPermanence){
        for( auto offset = 0 ; offset < owp.duration ; ++offset ){
            predictedObstacles[actualT + offset].insert(owp.loc);
        }
    }

    // get section
    SpawnedObstaclesSet sOSet;
    for(const auto& [t, obsSet] : predictedObstacles){
        for(const auto obs : obsSet) {
            sOSet.emplace(t - actualT, obs);
        }
    }
    return sOSet;
}

ObstaclesMap ObstaclesWrapper::getObstaclesFromJson(const nlohmann::json &obstaclesJson) {
    ObstaclesMap obstacles{};

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        TimeStep from = obsObj["t"];
        TimeStep to = from + static_cast<Interval>(obsObj["interval"]);

        for(auto t = from ; t < to ; ++t){
            obstacles[t].insert(static_cast<CompressedCoord>(obsObj["pos"]));
        }
    }

    return obstacles;
}

ProbabilitiesMap ObstaclesWrapper::getProbabilitiesFromJson(const nlohmann::json &obstaclesJson) {
    ProbabilitiesMap probabilitiesMap{};

    // one distribution for each obstacle
    const auto& pObj = obstaclesJson["probability"];

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        probabilitiesMap[obsObj["pos"]] = NormalInfo{pObj["mu"], pObj["sigma"]};
    }

    return probabilitiesMap;
}

ObstaclesWrapper::ObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson) :
    probabilitiesMap{getProbabilitiesFromJson(obstaclesJson)},
    trueObstacles{getObstaclesFromJson(obstaclesJson)},
    predictedObstacles{},
    gen{seed}
{}
