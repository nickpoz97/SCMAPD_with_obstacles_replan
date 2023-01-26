#include "Waypoint.hpp"
#include "Task.hpp"

Waypoint::operator CompressedCoord() const {return position;}

Waypoint::operator std::string() const {
    auto taskString = taskIndex.has_value() ? fmt::format("taskId: {}", taskIndex.value()) : "no task";
    return fmt::format("[pos: {}, demand: {}, {}]",
        position, static_cast<int>(demand), taskString);
}

Waypoint::Waypoint(CompressedCoord position, Demand demand, int taskIndex) :
    position(position),
    demand(demand),
    taskIndex(taskIndex) {}

TimeStep
Waypoint::update(TimeStep newArrivalTime, const std::vector<Task> &tasks, TimeStep previousCumulatedDelay) {
    arrivalTime = newArrivalTime;
    auto localDelay = 0;
    if (demand == Demand::DELIVERY) { arrivalTime.value() - tasks[taskIndex.value()].idealGoalTime; }
#ifndef NDEBUG
    if(localDelay < 0) { throw std::runtime_error("negative delay"); }
#endif
    cumulatedDelay = previousCumulatedDelay + localDelay;

    return cumulatedDelay;
}

TimeStep Waypoint::getCumulatedDelay() const {
    return cumulatedDelay;
}

TimeStep Waypoint::getArrivalTime() const {
    assert(arrivalTime.has_value());
    return arrivalTime.value();
}

Waypoint::Waypoint(CompressedCoord robotStartPosition) :
    position{robotStartPosition},
    demand{Demand::END},
    taskIndex{std::nullopt}
    {}

CompressedCoord Waypoint::getPosition() const {
    return position;
}

Demand Waypoint::getDemand() const {
    return demand;
}

int Waypoint::getTaskIndex() const {
    assert(taskIndex.has_value());
    return taskIndex.value();
}

Waypoint getTaskPickupWaypoint(const Task& task){
    return {task.startLoc, Demand::PICKUP, task.index};
}

Waypoint getTaskDeliveryWaypoint(const Task& task){
    return {task.goalLoc, Demand::DELIVERY, task.index};
}
