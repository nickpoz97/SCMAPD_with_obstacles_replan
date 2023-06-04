//
// Created by nicco on 04/06/2023.
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

struct NormalInfo{
    TimeStep mu;
    TimeStep std;
};

using ProbabilitiesMap = std::unordered_map<CompressedCoord, NormalInfo>;

struct ObstaclePersistence{
    CompressedCoord loc;
    Interval duration;
};

class ObstaclesWrapper {
public:
    ObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson, bool predict);
    SpawnedObstaclesSet updateAndGet(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions);
private:
    ProbabilitiesMap probabilitiesMap;
    ObstaclesMap trueObstacles;
    ObstaclesMap predictedObstacles;

    std::default_random_engine gen;

    bool predict;

    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);

    void updateWithPrediction(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles);
    void updateSimple(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles);
};


#endif //SIMULTANEOUS_CMAPD_OBSTACLESWRAPPER_HPP
