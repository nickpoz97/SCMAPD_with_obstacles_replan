//
// Created by nicco on 18/05/2023.
//

#ifndef SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP
#define SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP

#include <list>
#include <unordered_set>
#include "Coord.hpp"
#include "DistanceMatrix.hpp"

using CheckPoints = std::list<CompressedCoord>;

class RunningAgent {
public:
    RunningAgent(int agentId, Path plannedPath, CheckPoints plannedCheckpoints);

    [[nodiscard]] CompressedCoord getActualPosition() const;
    void stepAndUpdate();

    bool unexpectedObstacle(const std::unordered_set<CompressedCoord>& obstacles) const;

    const Path &getPlannedPath() const;

    void setPlannedPath(Path newPlannedPath);

    int getAgentId() const;

    const CheckPoints &getPlannedCheckpoints() const;

private:
    int agentId;
    Path plannedPath;
    CheckPoints plannedCheckpoints;
};

std::vector<RunningAgent> loadPlansFromJson(const nlohmann::json &j, const DistanceMatrix &dm);


#endif //SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP
