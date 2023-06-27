//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_ABSTRACTOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_ABSTRACTOBSTACLESWRAPPER_HPP

#include <unordered_map>
#include <unordered_set>
#include <random>
#include "Simulation/RunningAgent.hpp"
#include "NewTypes.hpp"
#include "Coord.hpp"
#include "SpawnedObstacle.hpp"

using ObstaclesMap = std::unordered_map<TimeStep , std::unordered_set<CompressedCoord>>;

struct ObstacleWithPersistence{
    CompressedCoord loc;
    TimeStep duration;
};

class AbstractObstaclesWrapper {
public:
    explicit AbstractObstaclesWrapper(ObstaclesMap obstaclesMap);
    virtual void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) = 0;
    virtual SpawnedObstaclesSet get() const = 0;

    virtual ~AbstractObstaclesWrapper() = default;
protected:
    ObstaclesMap obstaclesMap;

    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
};


#endif //SIMULTANEOUS_CMAPD_ABSTRACTOBSTACLESWRAPPER_HPP
