//
// Created by nicco on 13/01/2023.
//

#include <queue>
#include "MAPF/PathFinder.hpp"
#include "MAPF/Node.hpp"
#include "MAPF/ExploredSet.hpp"

static std::list<CompressedCoord> getPartialPath(
    const Status &status,
    int agentId,
    CompressedCoord startLoc,
    CompressedCoord goalLoc,
    TimeStep t
);

std::pair<Path, WaypointsList>
PathFinder::multiAStar(WaypointsList &&waypoints, CompressedCoord agentLoc, const Status &status, int agentId){
    if(waypoints.empty()){
        return {{agentLoc}, waypoints};
    }

    std::list<CompressedCoord> pathList{};

    auto actualLoc = agentLoc;
    TimeStep cumulatedDelay = 0;
    TimeStep t = 0;

    for(auto & w : waypoints){
        auto goalLoc = w.position;
        auto partialPath = getPartialPath(status, agentId, actualLoc, goalLoc, t);
        auto attachIt = pathList.empty() ? pathList.cend() :  std::prev(pathList.cend());
        pathList.splice(attachIt, partialPath);

        assert(!status.checkPathWithStatus({pathList.begin(), pathList.end()}, agentId));

        t = static_cast<int>(pathList.size()) - 1;
        // old goal is new start position
        actualLoc = goalLoc;

        cumulatedDelay = w.update(t, status.getTasks(), cumulatedDelay);
    }

    return {{pathList.begin(), pathList.end()}, waypoints};
}

static std::list<CompressedCoord> getPartialPath(const Status &status, int agentId, CompressedCoord startLoc, CompressedCoord goalLoc, TimeStep t) {
    ExploredSet exploredSet;
    const auto& dm = status.getDistanceMatrix();

    using NodePtr = std::shared_ptr<Node>;

    auto compareNodesPtr = [](const std::shared_ptr<Node>& nA, const std::shared_ptr<Node>& nB){
        return *nA > *nB;
    };

    std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(compareNodesPtr)> frontier;
    frontier.emplace(new Node{startLoc, t, dm.getDistance(startLoc, goalLoc)});

    const std::list<CompressedCoord> pathList{};

    while (!frontier.empty()){
        auto topNodePtr = frontier.top();
        exploredSet.add(*topNodePtr);
        frontier.pop();

        if(topNodePtr->getLocation() == goalLoc){
            return topNodePtr->getPathList();
        }

        auto neighbors = status.getValidNeighbors(agentId, topNodePtr->getLocation(), topNodePtr->getGScore());

        auto nextT = topNodePtr->getGScore() + 1;
        for(auto loc : neighbors){
            if(!exploredSet.contains(loc, nextT)){
                frontier.emplace(new Node{loc, nextT, dm.getDistance(loc, goalLoc), topNodePtr});
            }
        }
    }
    throw std::runtime_error("Path not found");
}

