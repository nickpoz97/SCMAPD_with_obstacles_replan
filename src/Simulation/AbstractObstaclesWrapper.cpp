//
// Created by nicco on 13/06/2023.
//

#include "Simulation/AbstractObstaclesWrapper.hpp"

double NormalInfo::getProb(int interval) const{
    auto dMu = static_cast<double>(mu);
    auto dStd = static_cast<double>(std);
    auto dX = static_cast<double>(interval);
    return (1 / (dStd * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((dX - dMu) / dStd, 2));
}

AbstractObstaclesWrapper::AbstractObstaclesWrapper(ProbabilitiesMap probabilitiesMap, ObstaclesMap obstaclesMap) :
    probabilitiesMap{std::move(probabilitiesMap)},
    obstaclesMap{std::move(obstaclesMap)}
{}

std::unordered_map<Interval, double> AbstractObstaclesWrapper::getProbabilities(CompressedCoord obsPos) const {
    const auto p = probabilitiesMap.at(obsPos);
    std::unordered_map<Interval , double> intervalP;

    for (int val = p.getLow() ; val <= p.getHigh() ; ++val){
        intervalP.emplace(val, p.getProb(val));
    }

    return intervalP;
}

ObstaclesMap AbstractObstaclesWrapper::getObstaclesFromJson(const nlohmann::json &obstaclesJson) {
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

ProbabilitiesMap AbstractObstaclesWrapper::getProbabilitiesFromJson(const nlohmann::json &obstaclesJson) {
    ProbabilitiesMap probabilitiesMap{};

    // one distribution for each obstacle
    const auto& pObj = obstaclesJson["probability"];

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        probabilitiesMap[obsObj["pos"]] = NormalInfo{pObj["mu"], pObj["sigma"]};
    }

    return probabilitiesMap;
}

AbstractObstaclesWrapper::~AbstractObstaclesWrapper() {}
