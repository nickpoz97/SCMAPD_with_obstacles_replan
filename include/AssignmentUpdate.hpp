#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENTUPDATE_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENTUPDATE_HPP

#include "Waypoint.hpp"
#include "Task.hpp"

namespace AssignmentUpdate{
    std::tuple<WaypointsList, Path, TimeStep> computeUpdatedResults(const WaypointsList& wpList, const Task& t);
    std::pair<Path, TimeStep> computeUpdatedResults(const WaypointsList& wpList);
}


#endif //SIMULTANEOUS_CMAPD_ASSIGNMENTUPDATE_HPP
