//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_OBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_OBSTACLESWRAPPER_HPP

#include <unordered_map>
#include <unordered_set>
#include <random>
#include "Simulation/RunningAgent.hpp"
#include "NewTypes.hpp"
#include "Coord.hpp"
#include "SpawnedObstacle.hpp"

using ObstaclesMap = std::unordered_map<TimeStep , std::unordered_set<CompressedCoord>>;

class ObstaclesWrapper {
public:
    explicit ObstaclesWrapper(const nlohmann::json &obstaclesJson);
    std::unordered_set<CompressedCoord> get(const std::vector<CompressedCoord> &nextPositions, TimeStep actualT) const;
protected:
    ObstaclesMap obstaclesMap;
    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
};


#endif //SIMULTANEOUS_CMAPD_OBSTACLESWRAPPER_HPP
