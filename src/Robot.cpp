//
// Created by nicco on 11/11/2022.
//

#include <numeric>
#include <limits>
#include "Robot.hpp"

Robot::Robot(CompressedCoord position, unsigned index, unsigned capacity) :
        startPosition{position},
        index{index},
        capacity{capacity}
    {}

void Robot::setTasksAndTTD(WaypointsList &&newWaypoints, TimeStep newTtd) {
    waypoints = std::move(newWaypoints);
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

const WaypointsList &Robot::getWaypoints() const {
    return waypoints;
}

TimeStep Robot::getTtd() const {
    return ttd;
}

unsigned Robot::getIndex() const {
    return index;
}

CompressedCoord Robot::getStartPosition() const {
    return startPosition;
}

bool Robot::empty() const {
    return waypoints.empty();
}

void Robot::setTasksAndTTD(const WaypointsList &newWaypoints, TimeStep newTtd) {
    waypoints = newWaypoints;
    ttd = newTtd;
}

void Robot::setTasksAndTTD(const Robot &robot) {
    setTasksAndTTD(robot.getWaypoints(), robot.getTtd());
}

void Robot::insert(const Task &task, Heuristic heuristic) {
    WaypointsList::iterator bestStartWaypoint, bestGoalWaypoint;
    TimeStep bestTTD = std::numeric_limits<TimeStep>::max();

    // search for best position for task start and goal
    for(auto waypointStart = waypoints.begin(); waypointStart != waypoints.end() ; ++waypointStart){
        for (auto waypointGoal = std::next(waypointStart); waypointGoal != waypoints.end(); ++waypointGoal){
            insertTaskWaypoints(task, waypointStart, waypointGoal);
            if(checkCapacityConstraint()){
                updateBestWaypoints(bestTTD, bestStartWaypoint, bestGoalWaypoint);
            }
            restorePreviousWaypoints(waypointStart, waypointGoal);
        }
    }

    waypoints.insert(bestStartWaypoint, {task.startLoc, Demand::START});
    waypoints.insert(bestGoalWaypoint, {task.goalLoc, Demand::GOAL});
}

void Robot::restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                     WaypointsList::iterator &waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

void Robot::insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                                WaypointsList::iterator &waypointGoal) {
    waypoints.insert(waypointStart, {task.startLoc, Demand::START});
    waypoints.insert(waypointGoal, {task.goalLoc, Demand::GOAL});
}

bool Robot::checkCapacityConstraint() {
    unsigned actualWeight = 0;

    for(const auto& waypoint : waypoints){
        actualWeight += static_cast<unsigned>(waypoint.second);
        if(actualWeight > getCapacity()){
            return false;
        }
    }
    return true;
}

TimeStep Robot::updateBestWaypoints(TimeStep bestTTD, WaypointsList::iterator &bestStart, WaypointsList::iterator &bestEnd) {
    // todo finish this
    return bestTTD;
}
