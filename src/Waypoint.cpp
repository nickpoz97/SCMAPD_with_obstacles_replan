#include "Waypoint.hpp"

Waypoint::operator Coord() const {return position;}

Waypoint::operator std::string() const {
        return fmt::format("[pos: {}, demand: {}, taskId: {}]",
            static_cast<std::string>(position), static_cast<int>(demand), taskIndex);
}

Waypoint::Waypoint(const Coord &position, Demand demand, int taskIndex) : position(position),
                                                                          demand(demand),
                                                                          taskIndex(taskIndex) {}

void Waypoint::setDelay(TimeStep t) {
    delay.emplace(t);
}

TimeStep Waypoint::getDelay() const {
    if(!delay.has_value()){
        throw std::runtime_error("Accessing timestep of path while not defined");
    }
    return delay.value();
}


