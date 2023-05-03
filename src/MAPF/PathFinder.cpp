//
// Created by nicco on 13/01/2023.
//

#include <queue>
#include "MAPF/PathFinder.hpp"
#include "MAPF/Node.hpp"

using NodeShrPtr = std::shared_ptr<Node>;

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

std::optional<Path>
PathFinder::multiAStar(const WaypointsList &waypoints, CompressedCoord agentLoc, const Status &status, int agentId){
    if(waypoints.empty()){
        throw std::runtime_error("No waypoints");
    }
    
    if(waypoints.size() == 1){
        assert(waypoints.cbegin()->getDemand() == Demand::END && waypoints.cbegin()->getPosition() == agentLoc);
        return Path{agentLoc};
    }

    assert(waypoints.crbegin()->getDemand() == Demand::END &&
        (waypoints.crbegin()->getPosition() == agentLoc || waypoints.crbegin()->getPosition() == std::next(waypoints.crbegin())->getPosition()));

    int i = 0;
    // <order, location>
    std::vector<std::pair<int, CompressedCoord>> goals;
    goals.reserve(waypoints.size());
    std::ranges::transform(
        waypoints,
        std::back_inserter(goals),
        [&i](const Waypoint& wp) -> std::pair<int, CompressedCoord> {return {i++, wp.getPosition()};}
    );

    // no home return
    if(goals.crbegin()->second == (goals.crbegin() + 1)->second){
        goals.erase(std::prev(goals.end()));
    }

    Frontier frontier;
    const auto &dm = status.getDistanceMatrix();
    int lastGoalIndex = std::ssize(goals) - 1;

    frontier.emplace(new Node{agentLoc, status.getTimeStep(), dm, *goals.cbegin()});
    ExploredSet exploredSet{};

    while (!frontier.empty()) {
        const auto topNode = frontier.top();
        frontier.pop();

        // do not explore this node
        if (exploredSet.contains(topNode)) {
            continue;
        }

        exploredSet.insert(topNode);

        // impossible to reach a location which is already occupied
        if(status.getPathWrappers().isAlreadyDocked(agentId, topNode->getTargetPosition(), topNode->getFScore())){
            return std::nullopt;
        }

        // entire path found
        if (topNode->getTargetIndex() == lastGoalIndex &&
            topNode->getLocation() == topNode->getTargetPosition() &&
            !status.dockingConflict(topNode->getGScore(), topNode->getTargetPosition(), agentId)) {
            auto pathList = topNode->getPathList();
            return Path{pathList.cbegin(), pathList.cend()};
        }

        auto neighbors = status.getValidNeighbors(agentId, topNode->getLocation(), topNode->getGScore(), true);

        auto nextT = topNode->getGScore() + 1;
        for (auto loc: neighbors) {
            NodeShrPtr neighbor {new Node{loc, nextT, dm, goals[topNode->getNextTargetIndex(lastGoalIndex)], topNode.get()}};
            if (!exploredSet.contains(neighbor)) {
                frontier.push(neighbor);
            }
        }
    }
    return std::nullopt;
}
