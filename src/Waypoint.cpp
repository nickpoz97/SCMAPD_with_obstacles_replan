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
Waypoint::updateCumulatedDelay(TimeStep arrivalTime, const std::vector<Task> &tasks, TimeStep previousCumulatedDelay) {
    // for performance reasons, if START the delay is set to 0
    auto localDelay = (demand == Demand::DELIVERY) ? arrivalTime - tasks[taskIndex].idealGoalTime : 0;
    cumulatedDelay = previousCumulatedDelay + localDelay;

    return cumulatedDelay;
}

TimeStep Waypoint::getCumulatedDelay() const {
    return cumulatedDelay;
}


