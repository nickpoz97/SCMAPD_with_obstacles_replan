//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ROBOT_HPP
#define SIMULTANEOUS_CMAPD_ROBOT_HPP

#include <TypeDefs.hpp>
#include "Task.hpp"

using Waypoints = std::list<CompressedCoord>;

class Robot {
public:
    explicit Robot(unsigned int capacity);

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] const Waypoints &getWaypoints() const;

    [[nodiscard]] TimeStep getTtd() const;

    void setTasksAndTTD(Waypoints &&newActions, TimeStep newTtd);
private:
    const unsigned int capacity;

    Waypoints waypoints{};
    TimeStep ttd = 0;

};


#endif //SIMULTANEOUS_CMAPD_ROBOT_HPP
