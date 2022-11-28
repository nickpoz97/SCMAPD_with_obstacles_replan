//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include "TypeDefs.hpp"
#include <Task.hpp>

struct Waypoint{
    CompressedCoord position;
    Demand demand;
    unsigned taskIndex;
};

using WaypointsList = std::list<Waypoint>;


#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
