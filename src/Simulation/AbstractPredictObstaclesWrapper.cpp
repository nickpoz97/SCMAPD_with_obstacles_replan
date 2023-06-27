//
// Created by nicco on 27/06/2023.
//

#include "Simulation/AbstractPredictObstaclesWrapper.hpp"

double NormalInfo::getProb(int interval) const{
    auto dMu = static_cast<double>(mu);
    auto dStd = static_cast<double>(std);
    auto dX = static_cast<double>(interval);
    return (1 / (dStd * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((dX - dMu) / dStd, 2));
}

std::unordered_map<TimeStep , double> AbstractPredictObstaclesWrapper::getProbabilities(CompressedCoord obsPos) const {
    const auto p = probabilitiesMap.at(obsPos);
    std::unordered_map<TimeStep  , double> intervalP;

    for (int val = p.getLow() ; val <= p.getHigh() ; val += p.std){
        intervalP.emplace(val, p.getProb(val));
    }

    return intervalP;
}

ProbabilitiesMap AbstractPredictObstaclesWrapper::getProbabilitiesFromJson(const nlohmann::json &obstaclesJson) {
    ProbabilitiesMap probabilitiesMap{};

    // one distribution for each obstacle
    const auto& pObj = obstaclesJson["probability"];

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        probabilitiesMap[obsObj["pos"]] = NormalInfo{pObj["mu"], pObj["sigma"]};
    }

    return probabilitiesMap;
}

AbstractPredictObstaclesWrapper::AbstractPredictObstaclesWrapper(ProbabilitiesMap probabilitiesMap,
                                                                 ObstaclesMap obstaclesMap) :
    AbstractObstaclesWrapper{std::move(obstaclesMap)},
    probabilitiesMap{std::move(probabilitiesMap)}
{}
