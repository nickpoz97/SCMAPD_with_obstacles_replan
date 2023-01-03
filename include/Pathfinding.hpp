#ifndef SIMULTANEOUS_CMAPD_PATHFINDING_HPP
#define SIMULTANEOUS_CMAPD_PATHFINDING_HPP

#include "Waypoint.hpp"
#include "Task.hpp"

namespace PathFinding{
    std::tuple<WaypointsList, Path, TimeStep> computeUpdatedResults(const WaypointsList& wpList, const Task& t);
    std::pair<Path, TimeStep> computeUpdatedResults(const WaypointsList& wpList);
}


#endif //SIMULTANEOUS_CMAPD_PATHFINDING_HPP
