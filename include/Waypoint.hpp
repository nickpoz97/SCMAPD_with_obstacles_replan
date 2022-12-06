//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include "TypeDefs.hpp"
#include "CMAPF/Point.h"

struct Waypoint{
    Coord position;
    Demand demand;
    unsigned taskIndex;

    inline operator Coord() const {return position;}
};

using WaypointsList = std::list<Waypoint>;


#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
