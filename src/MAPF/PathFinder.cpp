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
PathFinder::multiAStar(const std::vector<std::pair<int, CompressedCoord>>& goals, CompressedCoord agentLoc, const std::vector<Path> &paths,
        const AmbientMap &ambient, const SpawnedObstaclesSet& sOset) {
    const auto& dm = ambient.getDistanceMatrix();

    assert(!goals.empty());
    
    if(goals.size() == 1){
        assert(goals.front().second == agentLoc);
        return Path{agentLoc};
    }

    Frontier frontier;
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
            topNode->getLocation() == topNode->getTargetPosition()) {
            auto pathList = topNode->getPathList();
            return Path{pathList.cbegin(), pathList.cend()};
        }

        auto actualLoc = topNode->getLocation();
        auto neighbors = ambient.getNeighbors(actualLoc);

        auto actualT = topNode->getGScore();
        auto nextT = actualT + 1;
        auto target = topNode->getTargetPosition();
        bool isLastTarget = topNode->getTargetIndex() == lastGoalIndex;

        auto validNeighbors = neighbors |
            std::views::filter(
                [=, &paths, &sOset](CompressedCoord nb){
                    return std::ranges::none_of(
                        paths,
                        [=](const Path& p){
                            bool isFinal = (target == nb && isLastTarget);
                            return p.hasConflict(actualLoc, nb, actualT, isFinal);
                        }
                    ) && !sOset.contains({nextT, nb});
                }
            );

        for (auto loc: validNeighbors) {
            NodeShrPtr neighbor {new Node{loc, nextT, dm, goals[topNode->getNextTargetIndex(lastGoalIndex)], topNode.get()}};
            if (!exploredSet.contains(neighbor)) {
                frontier.push(neighbor);
            }
        }
    }
    return std::nullopt;
}

std::optional<Path>
PathFinder::multiAStar(const std::vector<std::pair<int, CompressedCoord>>& goals, CompressedCoord agentLoc, const std::vector<Path> &paths,
           const AmbientMap &ambient){
    return multiAStar(goals, agentLoc, paths, ambient, {});
}