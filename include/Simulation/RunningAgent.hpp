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

    [[nodiscard]] const Path &getPlannedPath() const;

    void setPlannedPath(Path newPlannedPath);

    [[nodiscard]] int getAgentId() const;

    [[nodiscard]] const CheckPoints &getPlannedCheckpoints() const;

    [[nodiscard]] bool hasFinished() const;

    std::optional<CompressedCoord> getNextPosition() const;

private:
    int agentId;
    Path plannedPath;
    CheckPoints plannedCheckpoints;
};

std::vector<RunningAgent> loadPlansFromJson(const nlohmann::json &j, const DistanceMatrix &dm);


#endif //SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP
