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

void Waypoint::updateDelay(TimeStep arrivalTime, const std::vector<Task> &tasks) {
    const Task& t = tasks[taskIndex];
    delay.emplace(demand == Demand::GOAL ? arrivalTime - t.idealGoalTime : arrivalTime - t.releaseTime);
}

TimeStep Waypoint::getDelay() const {
    if(!delay.has_value()){
        throw std::runtime_error("Accessing delay of waypoint while not defined");
    }
    return delay.value();
}


