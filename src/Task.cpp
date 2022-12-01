#include <Task.hpp>

bool operator==(const Task &t1, const Task &t2) {
        return t1.startLoc == t2.startLoc &&
               t1.goalLoc == t2.goalLoc &&
               t1.releaseTime == t2.releaseTime;
}

TimeStep Task::getIdealGoalTime(const DistanceMatrix &dm) const {
    return releaseTime + dm.getDistance(startLoc, goalLoc);
}
