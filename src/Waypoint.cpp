#include "Waypoint.hpp"
#include "Task.hpp"

Waypoint::operator CompressedCoord() const {return position;}

Waypoint::operator std::string() const {
    return fmt::format("[pos: {}, demand: {}, taskId: {}]",
        position, static_cast<int>(demand), taskIndex);
}

Waypoint::Waypoint(const CompressedCoord &position, Demand demand, int taskIndex) : position(position),
                                                                                    demand(demand),
                                                                                    taskIndex(taskIndex) {}

TimeStep
Waypoint::update(TimeStep newArrivalTime, const std::vector<Task> &tasks, TimeStep previousCumulatedDelay) {
    arrivalTime = newArrivalTime;
    auto localDelay = (demand == Demand::DELIVERY) ? newArrivalTime - tasks[taskIndex].idealGoalTime : 0;
    cumulatedDelay.emplace(previousCumulatedDelay + localDelay);

    return cumulatedDelay.value();
}

TimeStep Waypoint::getCumulatedDelay() const {
    return cumulatedDelay.value();
}

TimeStep Waypoint::getArrivalTime() const {
    return arrivalTime.value();
}

Waypoint getTaskPickupWaypoint(const Task& task){
    return {task.startLoc, Demand::PICKUP, task.index};
}

Waypoint getTaskDeliveryWaypoint(const Task& task){
    return {task.goalLoc, Demand::DELIVERY, task.index};
}
