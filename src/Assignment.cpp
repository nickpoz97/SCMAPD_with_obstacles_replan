#include <array>
#include <cassert>
#include <fstream>
#include <set>

#include "Assignment.hpp"
#include "MAPF/PathFinder.hpp"

Assignment::Assignment(const AgentInfo &agentInfo, int firstTaskId, const Status &status) :
        PathWrapper{{agentInfo.startPos}, {Waypoint{agentInfo.startPos}}, {}},
        index{agentInfo.index},
        capacity{agentInfo.capacity}
    {
        addTask(firstTaskId, status);
        assert(!getPath().empty() && *getPath().cbegin() == getStartPosition() && getWaypoints().size() == 3);
        assert(agentInfo.index == index);
    }

Assignment::Assignment(const AgentInfo &agentInfo, int newTaskId, const Status &status, const PathWrapper &pW) :
        PathWrapper{pW},
        index{agentInfo.index},
        capacity{agentInfo.capacity}
{
    addTask(newTaskId, status);
}

[[maybe_unused]] int Assignment::getCapacity() const {
    return capacity;
}

TimeStep Assignment::getMCA() const {
    return getActualTTD() - oldTTD;
}

int Assignment::getAgentId() const {
    return index;
}

CompressedCoord Assignment::getStartPosition() const {
    return PathWrapper::getInitialPos();
}

bool Assignment::empty() const {
    return getWaypoints().empty();
}

void
Assignment::addTask(int taskId, const Status &status) {

#ifndef NDEBUG
    auto oldWaypointSize = getWaypoints().size();
#endif
    // safe for NoPathException
    auto tmpOldTTD = getActualTTD();

    insertTaskWaypoints(status.getTask(taskId), status.getDistanceMatrix(), status.getTasks(), capacity);
    internalUpdate(status);

    oldTTD = tmpOldTTD;
    idealGoalTime = computeIdealGoalTime(status);

    satisfiedTasksIds.emplace(taskId);
#ifndef NDEBUG
    assert(oldWaypointSize == getWaypoints().size() - 2);
    int sum = 0;
    for(const auto& w : getWaypoints()){
        sum += static_cast<int>(w.getDemand());
    }
    assert(sum == 0);
    assert(
        std::ranges::all_of(
            getWaypoints().cbegin(),
            getWaypoints().cend(),
            [&](const Waypoint& wp){
                return wp.getDemand() == Demand::END || satisfiedTasksIds.contains(wp.getTaskIndex());
            }
        )
    );
    assert(
        std::ranges::all_of(
            getWaypoints().cbegin(),
            getWaypoints().cend(),
            [&](const Waypoint& wp){
                return std::find(getPath().cbegin(), getPath().cend(), wp.getPosition()) != getPath().cend();
            }
        )
    );
#endif
}

TimeStep Assignment::getActualTTD() const{
    assert(!getWaypoints().empty());
    return getWaypoints().crbegin()->getCumulatedDelay();
}

void
Assignment::internalUpdate(const Status &status) {
    PathAndWPsUpdate(PathFinder::multiAStar(getWaypoints(), getStartPosition(), status, index));

    assert(!status.hasIllegalPositions(getPath()));
    assert(!status.checkPathWithStatus(getPath(), index));
}

int operator<=>(const Assignment &a, const Assignment &b) {
    // signum function
    auto sgn = [](auto val){return (0 < val) - (val < 0);};

    int mcaScore = sgn(a.getMCA() - b.getMCA()) * 4;
    int pathSizeScore = sgn(std::ssize(a.getPath()) - std::ssize(b.getPath())) * 2;
    int idealSpanScore = sgn(a.getIdealGoalTime() - b.getIdealGoalTime());

    return mcaScore + pathSizeScore + idealSpanScore;
}

TimeStep Assignment::computeIdealGoalTime(const Status &status) const{
    assert(!getWaypoints().empty());
    TimeStep igt = 0;
    const auto& dm = status.getDistanceMatrix();

    igt += dm.getDistance(getStartPosition(), getWaypoints().cbegin()->getPosition());

    for(const auto& wp : getWaypoints()){
        if(wp.getDemand() == Demand::DELIVERY) { igt += status.getTask(wp.getTaskIndex()).idealGoalTime; }
    }

    if(getWaypoints().size() >= 3) {
        igt += dm.getDistance(getWaypoints().crbegin()->getPosition(), (std::next(getWaypoints().crbegin()))->getPosition());
    }

    return igt;
}

TimeStep Assignment::getIdealGoalTime() const {
    return idealGoalTime;
}

