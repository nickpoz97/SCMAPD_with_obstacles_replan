#include "PartialAssignment.hpp"

void PartialAssignment::setTasksAndTTD(Waypoints &&newWaypoints, TimeStep newTtd,
                                       std::forward_list<unsigned int> &&newTaskIndices) {
    demands = std::move(newTaskIndices);
    Robot::setTasksAndTTD(std::move(newWaypoints), newTtd);
}

void PartialAssignment::setTasksAndTTD(const Waypoints &newWaypoints, TimeStep newTtd,
                                       const std::forward_list<unsigned int> &newTaskIndices) {
    demands = newTaskIndices;
    Robot::setTasksAndTTD(newWaypoints, newTtd);
}

void PartialAssignment::insert(const Task& task) {
    auto waypointStart = waypoints.begin();
    auto waypointGoal = std::next(waypointStart);

    auto demandsStart = demands.begin();
    auto demandsGoal = std::next(demandsStart);

    auto bestStart = waypoints.begin();

    for(; waypointStart != waypoints.end() ; ++waypointStart, ++demandsStart){
        for (; waypointGoal != waypoints.end() ; ++waypointGoal, ++demandsGoal){
            insertTaskWaypoints(task, waypointStart, waypointGoal, demandsStart, demandsGoal);
            if(checkCapacityConstraint()){
                updateBestWaypoints();
            }
            restorePreviousWaypoints(waypointStart, waypointGoal, demandsStart, demandsGoal);
        }
    }
}

void PartialAssignment::restorePreviousWaypoints(std::list<CompressedCoord>::iterator &waypointStart,
                                                 std::list<CompressedCoord>::iterator &waypointGoal,
                                                 std::list<unsigned int>::iterator &demandsStart,
                                                 std::list<unsigned int>::iterator &demandsGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
    demands.erase(demandsStart);
    demands.erase(demandsGoal);
}

void PartialAssignment::insertTaskWaypoints(const Task &task, std::list<CompressedCoord>::iterator &waypointStart,
                                            std::list<CompressedCoord>::iterator &waypointGoal,
                                            std::list<unsigned int>::iterator &demandsStart,
                                            std::list<unsigned int>::iterator &demandsGoal) {
    waypoints.insert(waypointStart, task.startLoc);
    waypoints.insert(waypointGoal, task.goalLoc);
    demands.insert(demandsStart, 1);
    demands.insert(demandsGoal, -1);
}

bool PartialAssignment::checkCapacityConstraint() {
    unsigned actualWeight = 0;

    for(auto val : demands){
        actualWeight += val;
        if(actualWeight > getCapacity()){
            return false;
        }
    }
    return true;
}
