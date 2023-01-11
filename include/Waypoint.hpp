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
#include "Task.hpp"

struct Waypoint{
    const CompressedCoord position;
    const Demand demand;
    const int taskIndex;

    Waypoint(const CompressedCoord &position, Demand demand, int taskIndex);

    explicit operator CompressedCoord() const;

    explicit operator std::string() const;

    TimeStep update(TimeStep newArrivalTime, const std::vector<Task> &tasks, TimeStep previousCumulatedDelay);

    [[nodiscard]] TimeStep getCumulatedDelay() const;
    [[nodiscard]] TimeStep getArrivalTime() const;

private:
    std::optional<TimeStep> cumulatedDelay{};
    std::optional<TimeStep> arrivalTime{};
};

using WaypointsList = std::list<Waypoint>;

Waypoint getTaskPickupWaypoint(const Task& task);
Waypoint getTaskDeliveryWaypoint(const Task& task);

#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
