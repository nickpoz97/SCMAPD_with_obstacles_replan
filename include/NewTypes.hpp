#ifndef SIMULTANEOUS_CMAPD_NEWTYPES_HPP
#define SIMULTANEOUS_CMAPD_NEWTYPES_HPP

#include <vector>
#include <list>

using TimeStep = int;

enum class Demand : int{
    PICKUP = 1,
    DELIVERY = -1,
    END = 0
};

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

enum class Strategy{
    RETURN_TO_SPAWN,
    HOLD_AND_DODGE,
    WAIT_OTHERS
};

#endif //SIMULTANEOUS_CMAPD_NEWTYPES_HPP
