//
// Created by nicco on 17/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP

#include "AbstractSimulator.hpp"

struct PlanningResults{
    int score;
    std::vector<CompressedCoord> nextSteps;
};

class SmartSimulator : public AbstractSimulator{
public:
    SmartSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson,
        bool useMakeSpan);
private:
    bool useMakeSpan;

    [[nodiscard]] int getScore(const std::vector<CompressedCoord> &obstaclesPositions, bool useMakespan) const;
    size_t getResultPenalty(const vector<Path> &paths) const;

    PlanningResults simulateWithWait(const ObstaclesMap& obstaclesMap) const;
    PlanningResults simulateWithRePlan(const ObstaclesMap& obstaclesMap) const;
};


#endif //SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
