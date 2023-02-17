#ifndef SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP

#include <unordered_set>
#include "Coord.hpp"
#include "Waypoint.hpp"
#include "NewTypes.hpp"

struct PathWrapper{
    TimeStep ttd;
    TimeStep lastDeliveryTimeStep;
    Path path;
    WaypointsList wpList;
    std::unordered_set<int> satisfiedTasksIds;
};

struct ExtractedPath{
    int newTaskId;
    int agentId;
    PathWrapper wrapper;
};

#endif //SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
