//
// Created by nicco on 27/06/2023.
//

#include "Simulation/Predictor.hpp"

ProbabilitiesMap Predictor::getProbabilitiesFromJson(const nlohmann::json &obstaclesJson) {
    ProbabilitiesMap probabilitiesMap{};

    // one distribution for each obstacle
    const auto& pObj = obstaclesJson["probability"];

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        probabilitiesMap[obsObj["pos"]] = GaussInfo{pObj["mu"], pObj["sigma"]};
    }

    return probabilitiesMap;
}

Predictor::Predictor(const nlohmann::json &obstaclesJson, size_t seed) :
    probabilitiesMap{getProbabilitiesFromJson(obstaclesJson)},
    gen{seed}
{}

TimeStep Predictor::predict(CompressedCoord obsPos) const {
    auto gauss = probabilitiesMap.at(obsPos);
    std::normal_distribution<float> d(static_cast<float>(gauss.mu), static_cast<float>(gauss.std));
    return static_cast<TimeStep>(d(gen));
}

SpawnedObstaclesSet Predictor::predict(const std::unordered_set<CompressedCoord> &visibleObstacles) const {
    SpawnedObstaclesSet sOSet;

    for(auto cc : visibleObstacles){
        for(int t = 1 ; t <= predict(cc) ; ++t)
            sOSet.emplace(t, cc);
    }

    return sOSet;
}

std::vector<std::pair<TimeStep, double>> Predictor::getIntervalProbabilities(CompressedCoord pos) const{
    std::vector<std::pair<TimeStep, double>> intervalProbabilities{};
    auto gauss = probabilitiesMap.at(pos);

    for(auto interval = gauss.getLow() ; interval < gauss.getHigh() ; interval += gauss.std){
        intervalProbabilities.emplace_back(interval, gauss.getProb(interval));
    }
    return intervalProbabilities;
}

const GaussInfo & Predictor::getDistribution(CompressedCoord cc) const {
    return probabilitiesMap.at(cc);
}

SpawnedObstaclesSet Predictor::predictWithMemory(const std::unordered_set<CompressedCoord> &visibleObstacles) const {
    static SpawnedObstaclesSet cachedSOSet;

    for(auto sO : predict(visibleObstacles)){
        cachedSOSet.emplace(sO);
    }

    return cachedSOSet;
}

double GaussInfo::getProb(int interval) const {
    auto dMu = static_cast<double>(mu);
    auto dStd = static_cast<double>(std);
    auto dX = static_cast<double>(interval);
    return (1 / (dStd * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((dX - dMu) / dStd, 2));
}
