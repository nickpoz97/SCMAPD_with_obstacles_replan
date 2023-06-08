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

    [[nodiscard]] inline auto getProb(int x) const {
        auto dMu = static_cast<double>(mu);
        auto dStd = static_cast<double>(std);
        auto dX = static_cast<double>(x);
        return (1 / (dStd * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow((dX - dMu) / dStd, 2));
    }

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
private:
    ProbabilitiesMap probabilitiesMap;
    ObstaclesMap trueObstacles;
    ObstaclesMap predictedObstacles;

    std::default_random_engine gen;

    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);

    void updateWithPrediction(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles);
    void updateSimple(TimeStep actualT, const std::vector<CompressedCoord> &visibleObstacles);
};


#endif //SIMULTANEOUS_CMAPD_OBSTACLESWRAPPER_HPP
