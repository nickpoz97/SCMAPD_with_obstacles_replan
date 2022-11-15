#include "Task.hpp"


Task::Task(CompressedCoord startLoc, CompressedCoord goalLoc, TimeStep releaseTime) :
    startLoc{startLoc},
    goalLoc{goalLoc},
    releaseTime{releaseTime}
    {}

CompressedCoord Task::getStartLoc() const {
    return startLoc;
}

CompressedCoord Task::getGoalLoc() const {
    return goalLoc;
}

TimeStep Task::getReleaseTime() const {
    return releaseTime;
}
