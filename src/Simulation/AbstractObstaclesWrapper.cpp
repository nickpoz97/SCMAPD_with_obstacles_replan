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

AbstractObstaclesWrapper::AbstractObstaclesWrapper(const nlohmann::json &obstaclesJson) :
        probabilitiesMap{getProbabilitiesFromJson(obstaclesJson)},
        obstaclesMap{getObstaclesFromJson(obstaclesJson)}
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

bool AbstractObstaclesWrapper::newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const{
    const auto& gauss = probabilitiesMap.at(pos);
    auto interval = actualSpawnTime - firstSpawnTime;

    return interval > gauss.mu && gauss.getProb(interval) <= 0.01;
}

std::vector<CompressedCoord>
AbstractObstaclesWrapper::updateFoundObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t) {
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
