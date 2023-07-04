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
    void stepAndUpdate(TimeStep t);

    [[nodiscard]] const Path &getPlannedPath() const;

    void setPlannedPath(Path newPlannedPath);

    [[nodiscard]] int getAgentId() const;

    [[nodiscard]] const CheckPoints &getPlannedCheckpoints() const;

    [[nodiscard]] bool hasFinished() const;

    [[nodiscard]] CompressedCoord getNextPosition() const;

    [[nodiscard]] bool checkpointChecker(bool isWaiting) const;

    void forceWait();

    TimeStep getArrivalTimeStep() const;
private:
    int agentId;
    Path plannedPath;
    CheckPoints plannedCheckpoints;

    std::optional<CompressedCoord> cachedNextPosition;

    std::optional<TimeStep> arrivalTimeStep;
};

std::vector<RunningAgent> loadPlansFromJson(const nlohmann::json &j, const DistanceMatrix &dm);
std::size_t hash_value(const RunningAgent& s);
size_t computeSeed(const std::vector<RunningAgent>& runningAgentsVector);


#endif //SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP
