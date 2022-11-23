//
// Created by nicco on 11/11/2022.
//

#include <numeric>
#include "Robot.hpp"

Robot::Robot(CompressedCoord position, unsigned index, unsigned capacity) :
    position{position},
    index{index},
    capacity{capacity}
    {}

void Robot::setTasksAndTTD(Waypoints &&newActions, TimeStep newTtd) {
    waypoints = std::move(newActions);
    ttd = newTtd;
}

void Robot::setTasksAndTTD(Robot &&robot) {
    setTasksAndTTD(std::move(robot.waypoints), robot.ttd);
    robot.waypoints.clear();
    robot.ttd = 0;
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

unsigned Robot::getIndex() const {
    return index;
}

CompressedCoord Robot::getStartPosition() const {
    return position;
}

Waypoints Robot::releaseWaypoints() {
    return std::move(waypoints);
}

bool Robot::empty() const {
    return waypoints.empty();
}

void Robot::setTasksAndTTD(const Waypoints &newActions, TimeStep newTtd) {
    waypoints = newActions;
    ttd = newTtd;
}

void Robot::setTasksAndTTD(const Robot &robot) {
    setTasksAndTTD(robot.getWaypoints(), robot.getTtd());
}
