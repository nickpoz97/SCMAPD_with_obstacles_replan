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
    TimeStep duration;
};

class AbstractObstaclesWrapper {
public:
    explicit AbstractObstaclesWrapper(ProbabilitiesMap probabilitiesMap, ObstaclesMap obstaclesMap);
    virtual void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) = 0;
    virtual ObstaclesMap get() const = 0;

    std::unordered_map<TimeStep , double> getProbabilities(CompressedCoord obsPos) const;

    virtual ~AbstractObstaclesWrapper() = default;
protected:
    ProbabilitiesMap probabilitiesMap;
    ObstaclesMap obstaclesMap;

    static ObstaclesMap getObstaclesFromJson(const nlohmann::json &obstaclesJson);
    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);
};


#endif //SIMULTANEOUS_CMAPD_ABSTRACTOBSTACLESWRAPPER_HPP
