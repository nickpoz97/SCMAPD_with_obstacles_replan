//
// Created by nicco on 13/01/2023.
//

#include <queue>
#include "MAPF/PathFinder.hpp"
#include "MAPF/Node.hpp"

using NodeShrPtr = std::shared_ptr<Node>;

static std::optional<std::list<CompressedCoord>> getPartialPath(
    const Status &status,
    int agentId,
    CompressedCoord startLoc,
    CompressedCoord goalLoc,
    TimeStep t
);

template <>
struct std::hash<NodeShrPtr> {
    size_t operator()(const NodeShrPtr& node) const {
        size_t seed = 0;
        boost::hash_combine(seed, node->getLocation());
        boost::hash_combine(seed, node->getGScore());
        boost::hash_combine(seed, node->getTargetPosition());
        return seed;
    }
};

static void updateWaypointsStats(WaypointsList& waypointsList, const Path& path, const std::vector<Task>& tasksVector);

using Frontier = std::priority_queue<
    NodeShrPtr,
    std::vector<NodeShrPtr>,
    decltype([](const NodeShrPtr& a, const NodeShrPtr& b){return *a > *b;})
>;

using ExploredSet = std::unordered_set<
    NodeShrPtr,
    std::hash<NodeShrPtr>,
    decltype([](const NodeShrPtr& a, const NodeShrPtr& b){return *a == *b;})
>;

std::optional<std::pair<Path, WaypointsList>>
PathFinder::multiAStar(WaypointsList waypoints, CompressedCoord agentLoc, const Status &status, int agentId){
    if(waypoints.empty()){
        throw std::runtime_error("No waypoints");
    }

    // start position end end position should be the same
    if(waypoints.crbegin()->getDemand() != Demand::END || waypoints.crbegin()->getPosition() != agentLoc){
        throw std::runtime_error("Wrong end waypoint");
    }

    std::list<CompressedCoord> pathList;
    {
        int i = 0;
        // <order, location>
        std::vector<std::pair<int, CompressedCoord>> goals;
        goals.reserve(waypoints.size());
        std::ranges::transform(
            waypoints,
            std::back_inserter(goals),
            [&i](const Waypoint& wp) -> std::pair<int, CompressedCoord> {return {i++, wp.getPosition()};}
        );

        Frontier frontier;
        const auto &dm = status.getDistanceMatrix();
        int lastGoalIndex = std::ssize(goals) - 1;

        frontier.emplace(new Node{agentLoc, 0, dm, *goals.cbegin()});
        ExploredSet exploredSet{};

        while (!frontier.empty()) {
            const auto topNode = frontier.top();
            frontier.pop();

            // do not explore this node
            if (exploredSet.contains(topNode)) {
                continue;
            }
            exploredSet.insert(topNode);

            // entire path found
            if (topNode->getTargetIndex() == lastGoalIndex &&
                topNode->getLocation() == topNode->getTargetPosition() &&
                !status.dockingConflict(topNode->getGScore(), topNode->getTargetPosition(), agentId)) {
                pathList = topNode->getPathList();
                break;
            }

            //#ifndef NDEBUG
            //        auto agentsSnapshot = status.getAgentsSnapshot(
            //                agentId,
            //                topNodePtr->getGScore(),
            //                topNodePtr->getLocation()
            //        );
            //
            //        auto targetSnapshots = status.getTargetSnapshot(startLoc, goalLoc, topNodePtr->getLocation());
            //#endif

            auto neighbors = status.getValidNeighbors(agentId, topNode->getLocation(), topNode->getGScore(), true);

            auto nextT = topNode->getGScore() + 1;
            for (auto loc: neighbors) {
                NodeShrPtr neighbor {new Node{loc, nextT, dm, goals[topNode->getNextTargetIndex(lastGoalIndex)], topNode.get()}};
                if (!exploredSet.contains(neighbor)) {
                    frontier.push(neighbor);
                }
            }
        }
    }

    Path path{pathList.begin(), pathList.end()};
    updateWaypointsStats(waypoints, path, status.getTasks());

    return std::pair{path, waypoints};
}

static void updateWaypointsStats(WaypointsList& waypointsList, const Path& path, const std::vector<Task>& tasksVector){
    TimeStep cumulatedDelay = 0;
    auto wpIt = waypointsList.begin();

    for(int t = 0 ; t < path.size(); ++t){
        assert(wpIt != waypointsList.end());
        if (wpIt->getPosition() == path[t]){
            cumulatedDelay = wpIt->update(t, tasksVector, cumulatedDelay);
            ++wpIt;
        }
    }
}

