//
// Created by nicco on 03/01/2023.
//

#include "MAPF/MultiAStar.hpp"

Path MultiAStar::solve(const std::vector<Waypoint> &waypoints, const Coord &startLoc, const Status &status, int agentId) {
    if(waypoints.empty()){
        return {startLoc};
    }

    TimeStep t = 0;
    auto loc = status.toCompressedCoord(startLoc);
    auto wpIt = waypoints.cbegin();
    const Node * parent = nullptr;

    frontier.emplace(loc, t, status.getDistance(startLoc, wpIt->position));

    // todo complete this
    while (!frontier.empty()){

    }

    status.getValidNeighbors(0, startLoc, t);
}
