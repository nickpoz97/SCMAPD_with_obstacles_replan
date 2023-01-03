//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include <fmt/printf.h>
#include <optional>
#include "TypeDefs.hpp"
#include "Coord.hpp"
#include "utils.hpp"

struct Waypoint{
    const CompressedCoord position;
    const Demand demand;
    const int taskIndex;

    Waypoint(const CompressedCoord &position, Demand demand, int taskIndex);

    explicit operator CompressedCoord() const;

    explicit operator std::string() const;

    void setDelay(TimeStep t);

    [[nodiscard]] TimeStep getDelay() const;

private:
    std::optional<TimeStep> delay{};
};

using WaypointsList = std::list<Waypoint>;

#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
