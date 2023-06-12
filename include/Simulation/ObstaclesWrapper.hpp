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
    int mu;
    int std;

    [[nodiscard]] double getProb(int interval) const;

    [[nodiscard]] inline auto getLow() const{
        return mu - 3 * std;
    }

    [[nodiscard]] inline auto getHigh() const{
        return mu + 3 * std;
    }
};

using ProbabilitiesMap = std::unordered_map<CompressedCoord, NormalInfo>;

struct ObstaclePersistence{
    CompressedCoord loc;
    Interval duration;
};

class ObstaclesWrapper {
public:
    ObstaclesWrapper(size_t seed, const nlohmann::json &obstaclesJson);
    SpawnedObstaclesSet updateAndGet(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions, bool predict);

    std::unordered_map<Interval, double> getProbabilities(CompressedCoord obsPos) const;

    std::vector<CompressedCoord>
    updateFoundObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t);
private:
    ProbabilitiesMap probabilitiesMap;
    ObstaclesMap trueObstacles;
    ObstaclesMap predictedObstacles;

    std::default_random_engine gen;

    std::unordered_map<CompressedCoord, TimeStep> foundObstacles{};

    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);

    void updateWithPrediction(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles);
    void updateSimple(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles);

    bool newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const;
};


#endif //SIMULTANEOUS_CMAPD_OBSTACLESWRAPPER_HPP
