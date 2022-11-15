//
// Created by nicco on 11/11/2022.
//

#include <numeric>
#include "Robot.hpp"

Robot::Robot(CompressedCoord start, unsigned int capacity) :
    start{start},
    capacity{capacity}
    {}

CompressedCoord Robot::getStart() const {
    return start;
}

void Robot::setTasksAndTTD(Waypoints &&newActions, TimeStep newTtd) {
    waypoints = std::move(newActions);
    ttd = newTtd;
}

unsigned int Robot::getCapacity() const {
    return capacity;
}

const Waypoints &Robot::getWaypoints() const {
    return waypoints;
}

TimeStep Robot::getTtd() const {
    return ttd;
}
