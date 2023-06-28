//
// Created by nicco on 27/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_PREDICTOR_HPP
#define SIMULTANEOUS_CMAPD_PREDICTOR_HPP

#include <random>
#include "Coord.hpp"
#include "SpawnedObstacle.hpp"

struct GaussInfo{
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

using ProbabilitiesMap = std::unordered_map<CompressedCoord, GaussInfo>;

class Predictor {
public:
    Predictor(const nlohmann::json &obstaclesJson, size_t seed);
    TimeStep predict(CompressedCoord obsPos) const;
    SpawnedObstaclesSet predict(const std::unordered_set<CompressedCoord> &visibleObstacles) const;
    std::vector<std::pair<TimeStep, double>> getIntervalProbabilities(CompressedCoord pos) const;

    const GaussInfo & getDistribution(CompressedCoord cc) const;

private:
    ProbabilitiesMap probabilitiesMap;
    mutable std::default_random_engine gen;
    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);
};


#endif //SIMULTANEOUS_CMAPD_PREDICTOR_HPP
