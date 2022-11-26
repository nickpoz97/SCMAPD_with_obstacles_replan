//
// Created by nicco on 11/11/2022.
//

#include <numeric>
#include <limits>
#include "PartialAssignment.hpp"

void PartialAssignment::setTasks(WaypointsList &&newWaypoints, const TasksVector &tasks, const DistanceMatrix &distanceMatrix) {
    waypoints = std::move(newWaypoints);
    updatePath();
    ttd = computeRealTTD(tasks, distanceMatrix);
}

void PartialAssignment::setTasks(PartialAssignment &&pa, const TasksVector &tasks, const DistanceMatrix &distanceMatrix) {
    setTasks(std::move(pa.waypoints), tasks, distanceMatrix);
    pa.waypoints.clear();
    pa.ttd = 0;
}

const WaypointsList &PartialAssignment::getWaypoints() const {
    return waypoints;
}

bool PartialAssignment::empty() const {
    return waypoints.empty();
}

void PartialAssignment::setTasks(const WaypointsList &newWaypoints, const TasksVector &tasks, const DistanceMatrix &distanceMatrix) {
    waypoints = newWaypoints;
    updatePath();
    ttd = computeRealTTD(tasks, distanceMatrix);
}

void PartialAssignment::setTasks(const PartialAssignment &pa, const TasksVector &tasks, const DistanceMatrix &distanceMatrix) {
    setTasks(pa.getWaypoints(), tasks, distanceMatrix);
}

void
PartialAssignment::insert(const Task &task, Heuristic heuristic, const DistanceMatrix &distanceMatrix, const TasksVector &tasks) {
    WaypointsList::iterator bestStartIt, bestGoalIt;
    TimeStep bestApproxTTD = std::numeric_limits<TimeStep>::max();

    // search for best position for task start and goal
    for(auto waypointStart = waypoints.begin(); waypointStart != waypoints.end() ; ++waypointStart){
        for (auto waypointGoal = std::next(waypointStart); waypointGoal != waypoints.end(); ++waypointGoal){
            insertTaskWaypoints(task, waypointStart, waypointGoal);
            if(checkCapacityConstraint()){
                // todo add heuristic choices
                auto newApproxTtd = computeApproxTTD(tasks, distanceMatrix);
                if(newApproxTtd < bestApproxTTD){
                    bestApproxTTD = newApproxTtd;
                    bestStartIt = waypointStart;
                    bestGoalIt = waypointGoal;
                }
            }
            restorePreviousWaypoints(waypointStart, waypointGoal);
        }
    }

    waypoints.insert(bestStartIt, {task.startLoc, Demand::START, task.index});
    waypoints.insert(bestGoalIt, {task.goalLoc, Demand::GOAL, task.index});

    updatePath();
    ttd = computeRealTTD(tasks, distanceMatrix);
}

void PartialAssignment::restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                                 WaypointsList::iterator &waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

void PartialAssignment::insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                                            WaypointsList::iterator &waypointGoal) {
    waypoints.insert(waypointStart, {task.startLoc, Demand::START, task.index});
    waypoints.insert(waypointGoal, {task.goalLoc, Demand::GOAL, task.index});
}

bool PartialAssignment::checkCapacityConstraint() {
    unsigned actualWeight = 0;

    for(const auto& waypoint : waypoints){
        actualWeight += static_cast<unsigned>(waypoint.demand);
        if(actualWeight > getCapacity()){
            return false;
        }
    }
    return true;
}

TimeStep PartialAssignment::computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const{
    TimeStep cumulatedTTD = 0;
    auto wpIt = waypoints.cbegin();

    // i is the timestep
    for(int i = 0 ; i < path.size() ; ++i){
        // reached waypoint
        if(path[i] == wpIt->position){
            if(wpIt->demand == Demand::GOAL){
                const Task& task = tasks[wpIt->taskIndex];
                cumulatedTTD += i - task.releaseTime - distanceMatrix[task.startLoc][task.goalLoc];
            }
            std::next(wpIt);
        }
    }

    return cumulatedTTD;
}

TimeStep PartialAssignment::computeApproxTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const{
    TimeStep cumulatedTTD = 0;
    auto previousPos = startPosition;

    for(const auto& wp: waypoints){
        const Task& task = tasks[wp.taskIndex];
        if(wp.demand == Demand::GOAL){
            // simplified expression
            cumulatedTTD += distanceMatrix[previousPos][task.startLoc] - task.releaseTime;
            previousPos = task.goalLoc;
        }
    }

    return cumulatedTTD;
}

void PartialAssignment::updatePath() {
    // todo complete this
}

bool operator<(const PartialAssignment &a, const PartialAssignment &b) {
    return a.getTtd() < b.getTtd();
}
