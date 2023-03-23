#include <array>
#include <cassert>
#include <fstream>

#include "Assignment.hpp"
#include "MAPF/PathFinder.hpp"
#include "PathWrapper.hpp"

Assignment::Assignment(const AgentInfo &agentInfo) :
        PathWrapper{agentInfo},
        index{agentInfo.index}
    {}

TimeStep Assignment::getMCA() const {
    return mca;
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
    insertTaskWaypoints(status.getTask(taskId), status.getDistanceMatrix(), status.getTasks());

    if(!internalUpdate(status, true)){
        return false;
    }

    idealCost = computeIdealCost(status.getDistanceMatrix());
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
Assignment::internalUpdate(const Status &status, bool updateMCA) {
    // safe for NoPathException
    auto tmpOldTTD = getActualTTD();

    auto result{PathFinder::multiAStar(getWaypoints(), getInitialPos(), status, index)};
    if(!result.has_value()){
        return false;
    }

    pathAndWaypointsUpdate(std::move(result.value()));

    ttd = computeTTD(status.getTasks());
    realCost = std::next(getWaypoints().crbegin())->getRealArrivalTime();

    oldTTD = tmpOldTTD;
    mca = ttd - oldTTD;

    assert(!status.hasIllegalPositions(getPath()));
    assert(!status.checkPathWithStatus(getPath(), index));
    return true;
}

int operator<=>(const Assignment &a, const Assignment &b) {
    // signum function
    auto sgn = [](auto val){return (0 < val) - (val < 0);};

    int mcaScore = sgn(a.getMCA() - b.getMCA()) * 4;
    int pathSizeScore = sgn(a.getLastDeliveryTimeStep() - a.getLastDeliveryTimeStep()) * 2;
    int idealCost = sgn(a.getIdealCost() - b.getIdealCost());

    return mcaScore + pathSizeScore + idealCost;
}

Assignment &Assignment::operator=(const PathWrapper &otherPW) {
    PathWrapper::operator=(otherPW);
    return *this;
}
