#include <PathWrapper.hpp>
#include <utility>

TimeStep PWsVector::getMaxSpanCost() const {
    return std::max_element(
            cbegin(),
            cend(),
            [](const PathWrapper &pWA, const PathWrapper &pWB) {
                return pWA.getLastDeliveryTimeStep() <
                       pWB.getLastDeliveryTimeStep();
            }
    )->getLastDeliveryTimeStep();
}

TimeStep PWsVector::getTTD() const {
    return std::accumulate(
            cbegin(),
            cend(),
            0,
            [](TimeStep ttd, const PathWrapper& pW) {return ttd + pW.getTTD();}
    );
}

TimeStep PWsVector::getTTT() const {
    return std::accumulate(
            cbegin(),
            cend(),
            0,
            [](TimeStep ttt, const PathWrapper& pW) {return ttt + pW.getLastDeliveryTimeStep();}
    );
}

TimeStep PWsVector::getSpan(int agentId) const {
    return operator[](agentId).getLastDeliveryTimeStep();
}

TimeStep PWsVector::getTasksDelay(int agentId) const {
    return operator[](agentId).getTTD();
}

const Path& PWsVector::getPath(int agentId) const {
    return operator[](agentId).getPath();
}

bool PathWrapper::removeTasksAndWPs(const std::unordered_set<int> &rmvTasksIndices) {
    waypoints.remove_if([&](const Waypoint& wp){
        return wp.getDemand() != Demand::END && rmvTasksIndices.contains(wp.getTaskIndex());}
    );
    return std::erase_if(satisfiedTasksIds,[&](int taskId){return rmvTasksIndices.contains(taskId);});
}

TimeStep PathWrapper::getTTD() const {
    return waypoints.crbegin()->getCumulatedDelay();
}

TimeStep PathWrapper::getLastDeliveryTimeStep() const {
    if(waypoints.size() <= 1){
        return 0;
    }
    return std::next(waypoints.crbegin())->getArrivalTime();
}

const std::unordered_set<int> &PathWrapper::getSatisfiedTasksIds() const {
    return satisfiedTasksIds;
}

PathWrapper::PathWrapper(Path path, WaypointsList  wpList, std::unordered_set<int> satisfiedTasksIds) :
        path{std::move(path)},
        waypoints{std::move(wpList)},
        satisfiedTasksIds{std::move(satisfiedTasksIds)}
    {}

const WaypointsList &PathWrapper::getWaypoints() const {
    return waypoints;
}

WaypointsList &PathWrapper::getWaypoints() {
    return waypoints;
}

const Path &PathWrapper::getPath() const {
    return path;
}

CompressedCoord PathWrapper::getInitialPos() const {
    assert(!path.empty());
    return path[0];
}

void PathWrapper::update(std::pair<Path, WaypointsList> &&updatedData) {
    path = std::move(updatedData.first);
    waypoints = std::move(updatedData.second);

#ifndef NDEBUG
    assert(
        std::ranges::all_of(
                waypoints.cbegin(),
                waypoints.cend(),
                [&](const Waypoint& wp){
                return wp.getDemand() == Demand::END || satisfiedTasksIds.contains(wp.getTaskIndex());
            }
        )
    );
    assert(
        std::ranges::all_of(
                waypoints.cbegin(),
                waypoints.cend(),
                [&](const Waypoint& wp){
                return std::find(path.cbegin(), path.cend(), wp.getPosition()) != path.cend();
            }
        )
    );
#endif
}


