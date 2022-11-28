//
// Created by nicco on 28/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_WAYPOINT_HPP
#define SIMULTANEOUS_CMAPD_WAYPOINT_HPP

#include "TypeDefs.hpp"
#include <Task.hpp>

class Waypoint{
public:
    Waypoint(Demand demand, const Task& task);

    [[nodiscard]] CompressedCoord getPosition() const;

    [[nodiscard]] Demand getDemand() const;

    [[nodiscard]] unsigned int getTaskIndex() const;

    // return index of this waypoint in path
    TimeStep updateTTD(const Path& path, unsigned fromIndex);
private:
    CompressedCoord position;
    Demand demand;
    unsigned taskIndex;

    unsigned ttd = 0;
    // path from past wp to this one
    // Path path{};
};

using WaypointsList = std::list<Waypoint>;


#endif //SIMULTANEOUS_CMAPD_WAYPOINT_HPP
