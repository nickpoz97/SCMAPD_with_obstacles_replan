//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include <fmt/printf.h>
#include "TypeDefs.hpp"
#include "CMAPF/Point.h"
#include "utils.hpp"

struct Waypoint{
    Coord position;
    Demand demand;
    unsigned taskIndex;

    inline operator Coord() const {return position;}

    inline explicit operator std::string() const {
        return fmt::format("[pos: {}, demand: {}, taskId: {}]",
            utils::coordToString(position), static_cast<unsigned>(demand), taskIndex);
    }
};

using WaypointsList = std::list<Waypoint>;

#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
