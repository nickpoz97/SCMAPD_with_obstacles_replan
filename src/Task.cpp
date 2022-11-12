#include "Task.hpp"


Task::Task(CompressedCoord startLoc, CompressedCoord goalLoc, TimeStamp releaseTime) :
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

TimeStamp Task::getReleaseTime() const {
    return releaseTime;
}
