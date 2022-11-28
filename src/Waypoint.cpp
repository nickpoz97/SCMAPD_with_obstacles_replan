#include <Waypoint.hpp>
#include <stdexcept>

Waypoint::Waypoint(Demand demand, const Task &task) :
    position{demand == Demand::START ? task.startLoc : task.goalLoc},
    demand{demand},
    taskIndex{task.index}
    {}

CompressedCoord Waypoint::getPosition() const {
    return position;
}

Demand Waypoint::getDemand() const {
    return demand;
}

unsigned int Waypoint::getTaskIndex() const {
    return taskIndex;
}

TimeStep Waypoint::updateTTD(const Path &path, unsigned int fromIndex) {
    for (unsigned i = fromIndex ; i < path.size() ; ++i){
        if (path[i] == position){
            ttd = i - fromIndex;
            return i;
        }
    }
    throw std::runtime_error("Path does not contain waypoint");
}
