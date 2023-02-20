#include <PathWrapper.hpp>
#include <utility>

TimeStep PWsVector::getMaxSpanCost() const {
    return std::max_element(
            cbegin(),
            cend(),
            [](const PathWrapper& pWA, const PathWrapper& pWB){return pWA.getlastDeliveryTimeStep() < pWB.getlastDeliveryTimeStep();}
    )->getlastDeliveryTimeStep();
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
            [](TimeStep ttt, const PathWrapper& pW) {return ttt + pW.getlastDeliveryTimeStep();}
    );
}

TimeStep PWsVector::getSpan(int agentId) const {
    return operator[](agentId).getlastDeliveryTimeStep();
}

TimeStep PWsVector::getTasksDelay(int agentId) const {
    return operator[](agentId).getTTD();
}

const Path& PWsVector::getPath(int agentId) const {
    return operator[](agentId).getPath();
}

bool PathWrapper::removeTasksAndWPs(const std::unordered_set<int> &rmvTasksIndices) {
    wpList.remove_if([&](const Waypoint& wp){
        return wp.getDemand() != Demand::END && rmvTasksIndices.contains(wp.getTaskIndex());}
    );
    return std::erase_if(satisfiedTasksIds,[&](int taskId){return rmvTasksIndices.contains(taskId);});
}

TimeStep PathWrapper::getTTD() const {
    return wpList.crbegin()->getCumulatedDelay();
}

TimeStep PathWrapper::getlastDeliveryTimeStep() const {
    if(wpList.size() <= 1){
        return 0;
    }
    return std::next(wpList.crbegin())->getArrivalTime();
}

const std::unordered_set<int> &PathWrapper::getSatisfiedTasksIds() const {
    return satisfiedTasksIds;
}

PathWrapper::PathWrapper(Path path, WaypointsList  wpList, std::unordered_set<int> satisfiedTasksIds) :
    path{std::move(path)},
    wpList{std::move(wpList)},
    satisfiedTasksIds{std::move(satisfiedTasksIds)}
    {}

const WaypointsList &PathWrapper::getWaypoints() const {
    return wpList;
}

const Path &PathWrapper::getPath() const {
    return path;
}

CompressedCoord PathWrapper::getInitialPos() const {
    assert(!path.empty());
    return path[0];
}
