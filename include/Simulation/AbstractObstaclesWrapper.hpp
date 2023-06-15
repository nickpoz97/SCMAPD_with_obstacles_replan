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

struct ObstacleWithPersistence{
    CompressedCoord loc;
    Interval duration;
};

class AbstractObstaclesWrapper {
public:
    explicit AbstractObstaclesWrapper(const nlohmann::json &obstaclesJson);
    virtual void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) = 0;
    virtual ObstaclesMap get() const = 0;

    std::unordered_map<Interval, double> getProbabilities(CompressedCoord obsPos) const;

protected:
    ProbabilitiesMap probabilitiesMap;
    ObstaclesMap obstaclesMap;

    std::unordered_map<CompressedCoord, TimeStep> foundObstacles{};

    bool newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const;

    std::vector<CompressedCoord>
    updateFoundObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t);

    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);
};


#endif //SIMULTANEOUS_CMAPD_ABSTRACTOBSTACLESWRAPPER_HPP
