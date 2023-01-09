#ifndef SIMULTANEOUS_CMAPD_MULTIASTAR_HPP
#define SIMULTANEOUS_CMAPD_MULTIASTAR_HPP

#include <boost/heap/fibonacci_heap.hpp>
#include <set>
#include "Status.hpp"
#include "Node.hpp"
#include "Waypoint.hpp"
#include "ExploredSet.hpp"

using FrontierHeap = boost::heap::fibonacci_heap<Node, boost::heap::compare<std::greater<>>>;
using FrontierHandle = FrontierHeap::handle_type;
using FrontierHandlesMap = std::unordered_map<CompressedCoord, FrontierHandle>;

class MultiAStar {
public:
    MultiAStar() = default;
    std::pair<Path, WaypointsList>
    solve(WaypointsList &&waypoints, CompressedCoord agentLoc, const Status &status, int agentId);
private:
    static inline auto compareNodesPtr = [](const std::shared_ptr<Node>& nA, const std::shared_ptr<Node>& nB){
        return *nA < *nB;
    };

    ExploredSet exploredSet;
    std::set<std::shared_ptr<Node>, decltype(compareNodesPtr)> frontier;

    void updateFrontier(const std::shared_ptr<Node>& parentPtr, const std::vector<CompressedCoord> &neighbors, const DistanceMatrix &dm,
                        CompressedCoord targetPos);

    void fillPath(const Status &status, int agentId, CompressedCoord goalLoc, std::list<CompressedCoord> &pathList);
};


#endif //SIMULTANEOUS_CMAPD_MULTIASTAR_HPP
