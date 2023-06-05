//
// Created by nicco on 13/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_PATHFINDER_HPP
#define SIMULTANEOUS_CMAPD_PATHFINDER_HPP


#include <utility>
#include "MAPD/Waypoint.hpp"
#include "Coord.hpp"
#include "MAPD/Status.hpp"
#include "SpawnedObstacle.hpp"

namespace PathFinder{
    [[nodiscard]] std::optional<Path>
    multiAStar(const std::vector<std::pair<int, CompressedCoord>>& goals, CompressedCoord agentLoc, const std::vector<Path> &paths,
               const AmbientMap &ambient, const SpawnedObstaclesSet& sOset);

    [[nodiscard]] std::optional<Path>
    multiAStar(const std::vector<std::pair<int, CompressedCoord>>& goals, CompressedCoord agentLoc, const std::vector<Path> &paths,
               const AmbientMap &ambient);
}

#endif //SIMULTANEOUS_CMAPD_PATHFINDER_HPP
