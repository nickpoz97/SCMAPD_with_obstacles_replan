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
    SmartSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                   const nlohmann::json &obstaclesJson);
private:

    // <raId, wait>
    [[nodiscard]] std::unordered_map<int, bool> getBestChoices(const std::unordered_set<CompressedCoord> &visibleObstacles) const;

    [[nodiscard]] int computeNoObsScore(int raId) const;
    [[nodiscard]] int computeObsScore(CompressedCoord obsPos, int raId) const;

    int getScore(int raId, const vector<bool> &grid) const;
};


#endif //SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
