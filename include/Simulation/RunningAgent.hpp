//
// Created by nicco on 18/05/2023.
//

#ifndef SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP
#define SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP

#include <list>
#include "Coord.hpp"
#include "DistanceMatrix.hpp"

using PlannedPath = std::list<CompressedCoord>;
using PlannedCheckpoints = std::list<CompressedCoord>;

class RunningAgent {
public:
    RunningAgent(int agentId, PlannedPath plannedPath, PlannedCheckpoints plannedCheckpoints);

private:
    int agentId;
    PlannedPath plannedPath;
    PlannedCheckpoints plannedCheckpoints;
};

std::vector<RunningAgent> loadPlansFromJson(const nlohmann::json &j, const DistanceMatrix &dm);


#endif //SIMULTANEOUS_CMAPD_RUNNINGAGENT_HPP
