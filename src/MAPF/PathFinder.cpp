//
// Created by nicco on 13/01/2023.
//

#include <queue>
#include "MAPF/PathFinder.hpp"
#include "MAPF/Node.hpp"
#include "MAPF/ExploredSet.hpp"
#include "MAPF/NoPathException.hpp"
#include "MAPF/LimitedExploredSet.hpp"

static std::list<CompressedCoord> getPartialPath(
    const Status &status,
    int agentId,
    CompressedCoord startLoc,
    CompressedCoord goalLoc,
    TimeStep t
);

static void
holdPosition(
    const Status &status,
    int agentId,
    std::list<CompressedCoord> &pathList
);

std::pair<Path, WaypointsList>
PathFinder::multiAStar(WaypointsList waypoints, CompressedCoord agentLoc, const Status &status, int agentId){
    if(waypoints.empty()){
        throw std::runtime_error("No waypoints");
    }

    if(waypoints.crbegin()->getDemand() != Demand::END || waypoints.crbegin()->getPosition() != agentLoc){
        throw std::runtime_error("Wrong end waypoint");
    }

    std::list<CompressedCoord> pathList{agentLoc};

    auto actualLoc = agentLoc;
    TimeStep cumulatedDelay = 0;
    TimeStep t = 0;

    for(auto & w : waypoints){
        auto goalLoc = w.getPosition();
        auto partialPath = getPartialPath(status, agentId, actualLoc, goalLoc, t);

        // remove first pos (already in pathlist)
        partialPath.pop_front();

        pathList.splice(pathList.cend(), partialPath);

        t = static_cast<int>(pathList.size()) - 1;
        // old goal is new start position
        actualLoc = goalLoc;
#ifndef NDEBUG
        try {
#endif
            cumulatedDelay = w.update(t, status.getTasks(), cumulatedDelay);

#ifndef NDEBUG
        }
        catch (std::runtime_error& e){
            throw std::runtime_error("runtime error in waypoint catched");
        }
#endif
    }

    return {{pathList.begin(), pathList.end()}, waypoints};
}

static std::list<CompressedCoord> getPartialPath(const Status &status, int agentId, CompressedCoord startLoc, CompressedCoord goalLoc, TimeStep t) {
    auto maxPosVisits = status.getMaxPosVisits();

    std::unique_ptr<ExploredSet> exploredSet{
        maxPosVisits.has_value() ? new LimitedExploredSet{maxPosVisits.value()} : new ExploredSet
    };

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
        frontier.pop();

        // do not explore this node
        if(exploredSet->contains(*topNodePtr)){
            continue;
        }
        exploredSet->insert(*topNodePtr);

        // path found
        if(topNodePtr->getLocation() == goalLoc){
            return topNodePtr->getPathList();
        }

        auto neighbors = status.getValidNeighbors(agentId, topNodePtr->getLocation(), topNodePtr->getGScore(), true);

        auto nextT = topNodePtr->getGScore() + 1;
        for(auto loc : neighbors){
            if(!exploredSet->contains(loc, nextT)){
                frontier.emplace(new Node{loc, nextT, dm.getDistance(loc, goalLoc), topNodePtr});
            }
        }
    }
    throw NoPathException();
}

static void
holdPosition(const Status &status, int agentId, std::list<CompressedCoord> &pathList) {
    if(pathList.empty()){
        return;
    }

    auto totalTimeSteps = status.getLongestPathSize();
    TimeStep firstTimeStep = std::ssize(pathList) - 1;
    auto loc = *pathList.rbegin();

    // using t = 0 would not work
    for(int t = firstTimeStep ; t < totalTimeSteps ; ++t){
        auto nextLoc = status.holdOrAvailablePos(agentId, loc, t);
        pathList.push_back(nextLoc);
        loc = nextLoc;
    }
}
