#ifndef SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
#define SIMULTANEOUS_CMAPD_TYPEDEFS_HPP

#include <vector>
#include <list>
#include <queue>

#include "custom_types.h"

using Coord = cmapd::Point;
using CompressedCoord = unsigned int;
using TimeStep = unsigned int;

using PickupDropoffTime = std::pair<TimeStep, TimeStep>;

enum class Demand{
    START = 1,
    GOAL = -1
};

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

using Path = cmapd::path_t;

#endif //SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
