//
// Created by nicco on 13/06/2023.
//

#include "Simulation/RePlanObstaclesWrapper.hpp"

RePlanObstaclesWrapper::RePlanObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson) :
    AbstractObstaclesWrapper{obstaclesJson},
    gen{seed}
{}

ObstaclesMap RePlanObstaclesWrapper::get() const{
    return predictedObstacles;
}

void RePlanObstaclesWrapper::update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions){
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
            return {cc, static_cast<Interval>(d(gen))};
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
