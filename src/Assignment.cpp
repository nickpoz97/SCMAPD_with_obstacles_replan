#include <array>
#include <cassert>
#include <fstream>

#include "Assignment.hpp"
#include "MAPF/PathFinder.hpp"

Assignment::Assignment(const AgentInfo &agentInfo) :
        PathWrapper{{agentInfo.startPos}, {Waypoint{agentInfo.startPos}}, {}},
        index{agentInfo.index},
        capacity{agentInfo.capacity}
    {}

[[maybe_unused]] int Assignment::getCapacity() const {
    return capacity;
}

TimeStep Assignment::getMCA() const {
    return getTTD() - oldTTD;
}

int Assignment::getAgentId() const {
    return index;
}

bool Assignment::empty() const {
    return getWaypoints().empty();
}

bool
Assignment::addTask(int taskId, const Status &status) {

#ifndef NDEBUG
    auto oldWaypointSize = getWaypoints().size();
#endif
    // safe for NoPathException
    auto tmpOldTTD = getTTD();

    idealTTD = insertTaskWaypoints(status.getTask(taskId), status.getDistanceMatrix(), status.getTasks(), capacity);

    if(!internalUpdate(status)){
        return false;
    }

    oldTTD = tmpOldTTD;
    idealCost = computeIdealCost(status);

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

    return true;
}

bool
Assignment::internalUpdate(const Status &status) {
    auto result{PathFinder::multiAStar(getWaypoints(), getInitialPos(), status, index)};
    if(!result.has_value()){
        return false;
    }

    pathAndWaypointsUpdate(std::move(result.value()));

    oldTTD = status.getPathWrappers().getTasksDelay(getAgentId());

    assert(!status.hasIllegalPositions(getPath()));
    assert(!status.checkPathWithStatus(getPath(), index));
    return true;
}

int operator<=>(const Assignment &a, const Assignment &b) {
    // signum function
    auto sgn = [](auto val){return (0 < val) - (val < 0);};

    int mcaScore = sgn(a.getMCA() - b.getMCA());
    int pathSizeScore = sgn(a.getLastDeliveryTimeStep() - b.getLastDeliveryTimeStep());
    int idealCost = sgn(a.getIdealTTD() - b.getIdealTTD());

    return mcaScore * 4 + pathSizeScore * 2 + idealCost;
}

TimeStep Assignment::computeIdealCost(const Status &status) const{
    assert(!getWaypoints().empty());
    TimeStep cost = 0;
    const auto& dm = status.getDistanceMatrix();

    cost += dm.getDistance(getInitialPos(), getWaypoints().cbegin()->getPosition());
    auto prevPos = getInitialPos();
    for(const auto& wp: getWaypoints()){
        if(wp.getDemand() == Demand::END){
            break;
        }
        auto actualPos = wp.getPosition();
        cost += dm.getDistance(prevPos, actualPos);
        prevPos = actualPos;
    }

    return cost;
}

Assignment &Assignment::operator=(const PathWrapper &otherPW) {
    PathWrapper::operator=(otherPW);
    return *this;
}
