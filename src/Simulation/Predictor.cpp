//
// Created by nicco on 27/06/2023.
//

#include "Simulation/Predictor.hpp"

ProbabilitiesMap Predictor::getProbabilitiesFromJson(const nlohmann::json &obstaclesJson) {
    ProbabilitiesMap probabilitiesMap{};

    // one distribution for each obstacle
    const auto& probabilities = obstaclesJson["probability"];

    for(const auto& pInfo : probabilities){
        probabilitiesMap[pInfo["pos"]] = GaussInfo{pInfo["mu"], pInfo["sigma"]};
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
    return std::max(static_cast<TimeStep>(d(gen)), 1);
}

SpawnedObstaclesSet
Predictor::predictSimple(const std::unordered_set<CompressedCoord> &visibleObstacles, TimeStep actualT) const {
    SpawnedObstaclesSet sOSet;

    for(auto cc : visibleObstacles){
        if(cachedSOSet.contains({actualT, cc})){
            continue;
        }

        for(int t = 1 ; t <= predict(cc) ; ++t){
            sOSet.emplace(t, cc);
        }
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

SpawnedObstaclesSet
Predictor::predictWithMemory(const std::unordered_set<CompressedCoord> &visibleObstacles, TimeStep actualT) const {
    // relativeT is absolute in this case
    std::erase_if(cachedSOSet, [actualT](const SpawnedObstacle& sO){return sO.relativeT <= actualT;});

    for(auto sO : predictSimple(visibleObstacles, actualT)){
        cachedSOSet.emplace(sO.relativeT + actualT, sO.position);
    }

    auto normalizedSOSet = cachedSOSet | std::views::transform([actualT](const SpawnedObstacle& sO) -> SpawnedObstacle{
        return {sO.relativeT - actualT, sO.position};
    });

    return {normalizedSOSet.begin(), normalizedSOSet.end()};
}

double GaussInfo::getProb(int interval) const {
    auto dMu = static_cast<double>(mu);
    auto dStd = static_cast<double>(std);
    auto dX = static_cast<double>(interval);
    return (1 / (dStd * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((dX - dMu) / dStd, 2));
}
