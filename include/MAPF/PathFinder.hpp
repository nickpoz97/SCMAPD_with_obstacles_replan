//
// Created by nicco on 13/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_PATHFINDER_HPP
#define SIMULTANEOUS_CMAPD_PATHFINDER_HPP


#include <utility>
#include "Waypoint.hpp"
#include "Coord.hpp"
#include "Status.hpp"

namespace PathFinder{
    [[nodiscard]] std::optional<Path>
    multiAStar(const WaypointsList &waypoints, CompressedCoord agentLoc, const Status &status, int agentId);
}

#endif //SIMULTANEOUS_CMAPD_PATHFINDER_HPP
