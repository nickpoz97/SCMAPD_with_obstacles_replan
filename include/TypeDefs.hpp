#ifndef SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
#define SIMULTANEOUS_CMAPD_TYPEDEFS_HPP

#include <vector>
#include <list>
#include <queue>

using Coord = std::pair<unsigned int, unsigned int>;
using CompressedCoord = unsigned int;
using TimeStep = unsigned int;

using PickupDropoffTime = std::pair<TimeStep, TimeStep>;

using DistanceMatrix = std::vector<std::vector<TimeStep>>;

enum class Demand{
    START = 1,
    GOAL = -1
};

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

using Path = std::vector<CompressedCoord>;

#endif //SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
