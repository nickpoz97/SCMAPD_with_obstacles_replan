//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include "TypeDefs.hpp"
#include "CMAPF/Point.h"
#include <Task.hpp>

struct Waypoint{
    CompressedCoord position;
    Demand demand;
    unsigned taskIndex;

    [[nodiscard]] cmapd::Point toPoint(int nCols) const;
};

using WaypointsList = std::list<Waypoint>;


#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
