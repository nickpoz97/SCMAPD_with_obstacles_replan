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
#include <nlohmann/json.hpp>

struct Waypoint{
    Waypoint(const Task& task, Demand demand);
    explicit Waypoint(CompressedCoord robotStartPosition);

    explicit operator CompressedCoord() const;
    explicit operator std::string() const;

    TimeStep update(
        TimeStep newArrivalTime,
        TimeStep previousCumulatedDelay
    );

    [[nodiscard]] TimeStep getCumulatedDelay() const;
    [[nodiscard]] TimeStep getDelay() const;
    [[nodiscard]] TimeStep getArrivalTime() const;

    [[nodiscard]] CompressedCoord getPosition() const;

    [[nodiscard]] Demand getDemand() const;

    [[nodiscard]] int getTaskIndex() const;

    void setPosition(CompressedCoord newPos);

private:
    CompressedCoord position;
    Demand demand;
    std::optional<int> taskIndex;
    std::optional<TimeStep> idealGoalTime;

    TimeStep cumulatedDelay = 0;
    std::optional<TimeStep> arrivalTime{};
};

NLOHMANN_JSON_SERIALIZE_ENUM( Demand, {
    {Demand::PICKUP, "PICKUP"},
    {Demand::DELIVERY, "DELIVERY"},
    {Demand::END, "END"},
})

using WaypointsList = std::list<Waypoint>;

Waypoint getTaskPickupWaypoint(const Task& task);
Waypoint getTaskDeliveryWaypoint(const Task& task);
VerbosePath getWpCoords(const WaypointsList &wpList, const DistanceMatrix &dm);
nlohmann::json getWpsJson(const WaypointsList &wpList, const DistanceMatrix &dm);

#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
