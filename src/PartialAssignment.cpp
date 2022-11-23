#include "PartialAssignment.hpp"

void PartialAssignment::setTasksAndTTD(Waypoints &&newWaypoints, TimeStep newTtd,
                                       std::forward_list<unsigned int> &&newTaskIndices) {
    tasksIndices = std::move(newTaskIndices);
    Robot::setTasksAndTTD(std::move(newWaypoints), newTtd);
}

void PartialAssignment::setTasksAndTTD(const Waypoints &newWaypoints, TimeStep newTtd,
                                       const std::forward_list<unsigned int> &newTaskIndices) {
    tasksIndices = newTaskIndices;
    Robot::setTasksAndTTD(newWaypoints, newTtd);
}

const std::forward_list<unsigned int> &PartialAssignment::getTasksIndices() const {
    return tasksIndices;
}
