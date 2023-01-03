#ifndef SIMULTANEOUS_CMAPD_MULTIASTAR_HPP
#define SIMULTANEOUS_CMAPD_MULTIASTAR_HPP

#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_set>
#include "Status.hpp"
#include "Node.hpp"
#include "Waypoint.hpp"

class MultiAStar {
public:
    MultiAStar() = default;
    Path solve(const std::vector<Waypoint> &waypoints, const Coord &startLoc, const Status &status, int agentId);
private:
    auto nodeHasher = [](const Node& n){return n.;}

    WaypointsList wp;
    std::unordered_set<Node, > exploredSet;
    boost::heap::fibonacci_heap<Node, boost::heap::compare<std::greater<>>> frontier;
};


#endif //SIMULTANEOUS_CMAPD_MULTIASTAR_HPP
