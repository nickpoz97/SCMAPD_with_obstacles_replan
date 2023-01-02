#ifndef SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
#define SIMULTANEOUS_CMAPD_TYPEDEFS_HPP

#include <vector>
#include <list>

using TimeStep = int;

using PickupDropoffTime = std::pair<TimeStep, TimeStep>;

enum class Demand : int{
    START = 1,
    GOAL = -1
};

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

#endif //SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
