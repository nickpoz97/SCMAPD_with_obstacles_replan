//
// Created by nicco on 04/06/2023.
//

#include "Simulation/ObstaclesWrapper.hpp"

SpawnedObstaclesSet
ObstaclesWrapper::updateAndGet(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions, bool predict) {
    // no more interesting ones removed
    std::erase_if(predictedObstacles, [actualT](const auto& kv){return kv.first <= actualT;});
    std::erase_if(trueObstacles, [actualT](const auto& kv){return kv.first < actualT;});

    // obtain obstacles found by agents and get their predicted persistence
    auto visibleObstacles = nextPositions |
       std::views::filter([&](CompressedCoord cc){return trueObstacles[actualT].contains(cc);});

    if(predict){
        updateWithPrediction(actualT, {visibleObstacles.begin(), visibleObstacles.end()});
    }
    else{
        updateSimple(actualT, {visibleObstacles.begin(), visibleObstacles.end()});
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

void ObstaclesWrapper::updateWithPrediction(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles) {
    auto obstaclesWithPermanence = visibleObstacles |
                                   std::views::transform([&](CompressedCoord cc) -> ObstaclePersistence{
            auto [mu, std] = probabilitiesMap[cc];
            std::normal_distribution<float> d(static_cast<float>(mu), static_cast<float>(std));
            return {cc, static_cast<Interval>(d(gen))};
        }
    );

    // update section
    for(const auto& owp : obstaclesWithPermanence){
        for( auto offset = 1 ; offset <= owp.duration ; ++offset ){
            predictedObstacles[actualT + offset].insert(owp.loc);
        }
    }
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

void ObstaclesWrapper::updateSimple(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles) {
    // update section
    for(const auto cc : visibleObstacles){
        predictedObstacles[actualT + 1].insert(cc);
    }
}

std::unordered_map<Interval, double> ObstaclesWrapper::getProbabilities(CompressedCoord obsPos) const {
    const auto p = probabilitiesMap.at(obsPos);
    std::unordered_map<Interval , double> intervalP;

    for (int val = p.getLow() ; val <= p.getHigh() ; ++val){
        intervalP.emplace(val, p.getProb(val));
    }

    return intervalP;
}

bool ObstaclesWrapper::newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const{
    const auto& gauss = probabilitiesMap.at(pos);
    auto interval = actualSpawnTime - firstSpawnTime;

    return interval > gauss.mu && gauss.getProb(interval) <= 0.01;
}

std::vector<CompressedCoord>
ObstaclesWrapper::updateFoundObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t) {
    std::vector<CompressedCoord> newObstaclesPos;

    for(auto pos : obstaclesPositions){
        // update if necessary
        if(!foundObstacles.contains(pos) || newAppearance(pos, foundObstacles[pos], t)){
            foundObstacles[pos] = t;
            newObstaclesPos.push_back(pos);
        }
    }

    return newObstaclesPos;
}

double NormalInfo::getProb(int interval) const{
    auto dMu = static_cast<double>(mu);
    auto dStd = static_cast<double>(std);
    auto dX = static_cast<double>(interval);
    return (1 / (dStd * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((dX - dMu) / dStd, 2));
}
