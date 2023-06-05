//
// Created by nicco on 13/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_PATHFINDER_HPP
#define SIMULTANEOUS_CMAPD_PATHFINDER_HPP


#include <utility>
#include "MAPD/Waypoint.hpp"
#include "Coord.hpp"
#include "MAPD/Status.hpp"

namespace PathFinder{
    [[nodiscard]] std::optional<Path>
    multiAStar(const WaypointsList &waypoints, CompressedCoord agentLoc, const std::vector<Path> &paths,
               const AmbientMap &ambient);
}

#endif //SIMULTANEOUS_CMAPD_PATHFINDER_HPP
