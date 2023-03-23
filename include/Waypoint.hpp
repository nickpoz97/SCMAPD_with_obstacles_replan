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
    Waypoint(CompressedCoord position, Demand demand, int taskIndex);
    explicit Waypoint(CompressedCoord robotStartPosition);

    explicit operator std::string() const;

    TimeStep update(TimeStep newArrivalTime);

    [[nodiscard]] TimeStep getDelay(const std::vector<Task>& tasks) const;
    [[nodiscard]] TimeStep getRealArrivalTime() const;

    [[nodiscard]] CompressedCoord getPosition() const;
    [[nodiscard]] Demand getDemand() const;
    [[nodiscard]] int getTaskIndex() const;

    void reset();
private:
    CompressedCoord position;
    Demand demand;
    std::optional<int> taskIndex;
    std::optional<TimeStep> realArrivalTime;
};

NLOHMANN_JSON_SERIALIZE_ENUM( Demand, {
    {Demand::PICKUP, "PICKUP"},
    {Demand::DELIVERY, "DELIVERY"},
    {Demand::END, "END"},
})

class WaypointsList : public std::list<Waypoint>{
public:
    void reset();
};

Waypoint getTaskPickupWaypoint(const Task& task);
Waypoint getTaskDeliveryWaypoint(const Task& task);
VerbosePath getWpCoords(const WaypointsList &wpList, const DistanceMatrix &dm);
nlohmann::json getWpsJson(const WaypointsList &wpList, const DistanceMatrix &dm);

#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
