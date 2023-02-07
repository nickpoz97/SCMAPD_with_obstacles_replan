//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include <fmt/printf.h>
#include <optional>
#include "NewTypes.hpp"
#include "Coord.hpp"
#include "utils.hpp"
#include "Task.hpp"

struct Waypoint{
    Waypoint(CompressedCoord position, Demand demand, int taskIndex);
    explicit Waypoint(CompressedCoord robotStartPosition);

    explicit operator CompressedCoord() const;
    explicit operator std::string() const;

    TimeStep update(
        TimeStep newArrivalTime,
        const std::vector<Task> &tasks,
        TimeStep previousCumulatedDelay
    );

    [[nodiscard]] TimeStep getCumulatedDelay() const;
    [[nodiscard]] TimeStep getArrivalTime() const;

    [[nodiscard]] CompressedCoord getPosition() const;

    [[nodiscard]] Demand getDemand() const;

    [[nodiscard]] int getTaskIndex() const;

private:
    const CompressedCoord position;
    const Demand demand;
    const std::optional<int> taskIndex;

    TimeStep cumulatedDelay = 0;
    std::optional<TimeStep> arrivalTime{};
};

using WaypointsList = std::list<Waypoint>;

Waypoint getTaskPickupWaypoint(const Task& task);
Waypoint getTaskDeliveryWaypoint(const Task& task);

#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
