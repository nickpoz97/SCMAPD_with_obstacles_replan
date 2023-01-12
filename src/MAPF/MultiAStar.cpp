//
// Created by nicco on 03/01/2023.
//

#include "MAPF/MultiAStar.hpp"

std::pair<Path, WaypointsList>
MultiAStar::solve(WaypointsList &&waypoints, CompressedCoord agentLoc, const Status &status, int agentId) {
    if(waypoints.empty()){
        return {{agentLoc}, waypoints};
    }

    std::list<CompressedCoord> pathList{};

    auto actualLoc = agentLoc;

    TimeStep cumulatedDelay = 0;

    const auto& distanceMatrix = status.getDistanceMatrix();

    TimeStep t = 0;

    // todo fix possible loop
    for(auto & w : waypoints){
        auto goalLoc = w.position;

        frontier.emplace(new Node{actualLoc, t, distanceMatrix.getDistance(actualLoc, goalLoc)});
        t = fillPath(status, agentId, goalLoc, pathList);
        assert(!status.checkPathWithStatus({pathList.begin(), pathList.end()}, agentId));

        frontier.clear();
        exploredSet.clear();

        // old goal is new start position
        actualLoc = goalLoc;
        cumulatedDelay = w.update(t, status.getTasks(), cumulatedDelay);
    }

    return {{pathList.begin(), pathList.end()}, waypoints};
}

TimeStep MultiAStar::fillPath(const Status &status, int agentId, CompressedCoord goalLoc, std::list<CompressedCoord> &pathList) {
    while (!frontier.empty()){
        auto topNodePtr = *frontier.cbegin();
        frontier.erase(frontier.cbegin());

        if(topNodePtr->getLocation() == goalLoc){
            auto partialPathList = topNodePtr->getPathList();
            // merge (without std::prev there would be duplicates)
            auto attachIt = pathList.empty() ? pathList.cend() :  std::prev(pathList.cend());
            pathList.splice(attachIt, partialPathList);
            return topNodePtr->getGScore();
        }

        auto neighbors = status.getValidNeighbors(agentId, topNodePtr->getLocation(), topNodePtr->getGScore());

        updateFrontier(topNodePtr, neighbors, status.getDistanceMatrix(), goalLoc);

        exploredSet.add(*topNodePtr);
    }
    throw std::runtime_error("Path not found");
}

void
MultiAStar::updateFrontier(const std::shared_ptr<Node>& parentPtr, const std::vector<CompressedCoord> &neighbors, const DistanceMatrix &dm,
                           CompressedCoord targetPos) {
    auto newT = parentPtr->getGScore() + 1;
    for(auto loc : neighbors){
        if(!exploredSet.contains(loc, newT)){
            frontier.emplace(new Node{loc, newT, dm.getDistance(loc, targetPos), parentPtr});
        }
    }
}
