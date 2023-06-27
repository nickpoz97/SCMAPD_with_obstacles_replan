//
// Created by nicco on 13/06/2023.
//

#include "Simulation/PredictedObstaclesWrapper.hpp"

PredictedObstaclesWrapper::PredictedObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson) :
    AbstractPredictObstaclesWrapper(getProbabilitiesFromJson(obstaclesJson), getObstaclesFromJson(obstaclesJson)),
    gen{seed}
{}

PredictedObstaclesWrapper::PredictedObstaclesWrapper(size_t seed, ObstaclesMap obstaclesMap, ProbabilitiesMap probabilitiesMap) :
    AbstractPredictObstaclesWrapper(std::move(probabilitiesMap), std::move(obstaclesMap)),
    gen{seed}
{}

SpawnedObstaclesSet PredictedObstaclesWrapper::get() const{
    SpawnedObstaclesSet sOSet;

    for(const auto& kv : predictedObstacles){
        auto relativeT = kv.first - actualTimeStep;

        for(const auto& v : kv.second){
            sOSet.emplace(relativeT, v);
        }
    }

    return sOSet;
}

void PredictedObstaclesWrapper::update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions){
    assert(actualTimeStep <= actualT);
    actualTimeStep = actualT;

    auto obstaclesWithPermanence =
        nextPositions |
        // not predict again if present
        std::views::filter([&](CompressedCoord cc){
            return obstaclesMap[actualT].contains(cc) && !predictedObstacles[actualT+1].contains(cc);
        }) |
        // prediction
        std::views::transform([&](CompressedCoord cc) -> ObstacleWithPersistence{
            auto [mu, std] = probabilitiesMap[cc];
            std::normal_distribution<float> d(static_cast<float>(mu), static_cast<float>(std));
            return {cc, static_cast<TimeStep>(d(gen))};
        });

    // erase not interesting obstacles
    std::erase_if(predictedObstacles, [actualT](const auto& kv){return kv.first <= actualT;});

    // update section
    for(const auto& owp : obstaclesWithPermanence){
        for( auto offset = 1 ; offset <= owp.duration ; ++offset ){
            predictedObstacles[actualT + offset].insert(owp.loc);
        }
    }
}
