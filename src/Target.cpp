#include "Target.hpp"


Target::Target(LocationIndex startLoc, LocationIndex goalLoc, TimeStamp releaseTime) :
    startLoc{startLoc},
    goalLoc{goalLoc},
    releaseTime{releaseTime}
    {}

LocationIndex Target::getStartLoc() const {
    return startLoc;
}

LocationIndex Target::getGoalLoc() const {
    return goalLoc;
}

TimeStamp Target::getReleaseTime() const {
    return releaseTime;
}
