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

    auto startLoc = agentLoc;

    TimeStep cumulatedDelay = 0;

    const auto& distanceMatrix = status.getDistanceMatrix();

    // todo check this (due to code complexity)
    for(auto & w : waypoints){
        auto goalLoc = w.position;

        frontier.emplace(new Node{startLoc, 0, distanceMatrix.getDistance(startLoc, goalLoc)});
        fillPath(status, agentId, goalLoc, pathList);

        frontier.clear();
        exploredSet.clear();

        startLoc = goalLoc;
        TimeStep t = pathList.size() - 1;
        cumulatedDelay = w.update(t, status.getTasks(), cumulatedDelay);
    }

    return {{pathList.begin(), pathList.end()}, waypoints};
}

void MultiAStar::fillPath(const Status &status, int agentId, CompressedCoord goalLoc, std::list<CompressedCoord> &pathList) {
    while (!frontier.empty()){
        auto topNodePtr = *frontier.cbegin();
        frontier.erase(frontier.cbegin());

        if(topNodePtr->getLocation() == goalLoc){
            auto partialPathList = topNodePtr->getPathList();
            // merge
            pathList.splice(pathList.end(), partialPathList);
            return;
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
