//
// Created by nicco on 03/01/2023.
//

#include "MAPF/MultiAStar.hpp"

std::tuple<Path, TimeStep, WaypointsList>
MultiAStar::solve(WaypointsList &&waypoints, CompressedCoord agentLoc, const Status &status, int agentId) {
    if(waypoints.empty()){
        return {{agentLoc}, 0};
    }
    std::list<CompressedCoord> pathList{};

    auto startLoc = agentLoc;
    TimeStep t = 0;

    TimeStep ttd = 0;

    // todo check this (due to code complexity)
    for(auto & w : waypoints){
        auto goalLoc = w.position;

        frontier.emplace(new Node{startLoc, t, status.getDistance(startLoc, goalLoc)});
        fillPath(status, agentId, goalLoc, pathList);

        frontier.clear();
        exploredSet.clear();

        startLoc = goalLoc;
        t = pathList.size() - 1;
        w.updateDelay(t, status.getTasks());

        if (w.demand == Demand::GOAL){
            ttd += w.getDelay();
        }
    }

    return {{pathList.begin(), pathList.end()}, ttd, waypoints};
}

void MultiAStar::fillPath(const Status &status, int agentId, CompressedCoord goalLoc, std::list<CompressedCoord> &pathList) {
    while (!frontier.empty()){
        auto topNodePtr = *frontier.cbegin();
        frontier.erase(frontier.cbegin());

        if(topNodePtr->getLocation() == goalLoc){
            auto partialPathList = topNodePtr->getPathList();
            // merge
            pathList.splice(pathList, partialPathList);
            return;
        }

        auto neighbors = status.getValidNeighbors(agentId, topNodePtr->getLocation(), topNodePtr->getGScore());

        updateFrontier(topNodePtr, neighbors, status, goalLoc);

        exploredSet.add(*topNodePtr);
    }
    throw std::runtime_error("Path not found");
}

void
MultiAStar::updateFrontier(const std::shared_ptr<Node>& parentPtr, const std::vector<CompressedCoord> &neighbors, const Status &status,
                           CompressedCoord targetPos) {
    auto newT = parentPtr->getGScore() + 1;
    for(auto loc : neighbors){
        if(!exploredSet.contains(loc, newT)){
            frontier.emplace(new Node{loc, newT, status.getDistance(loc, targetPos), parentPtr});
        }
    }
}
