#include <PathWrapper.hpp>

TimeStep PWsVector::getMaxSpanCost() const {
    return std::max_element(
            cbegin(),
            cend(),
            [](const PathWrapper& pWA, const PathWrapper& pWB){return pWA.lastDeliveryTimeStep < pWB.lastDeliveryTimeStep;}
    )->lastDeliveryTimeStep;
}

TimeStep PWsVector::getTTD() const {
    return std::accumulate(
            cbegin(),
            cend(),
            0,
            [](TimeStep ttd, const PathWrapper& pW) {return ttd + pW.ttd;}
    );
}

TimeStep PWsVector::getTTT() const {
    return std::accumulate(
            cbegin(),
            cend(),
            0,
            [](TimeStep ttt, const PathWrapper& pW) {return ttt + pW.lastDeliveryTimeStep;}
    );
}

TimeStep PWsVector::getSpan(int agentId) const {
    return operator[](agentId).lastDeliveryTimeStep;
}

TimeStep PWsVector::getTasksDelay(int agentId) const {
    return operator[](agentId).ttd;
}

const Path& PWsVector::getPath(int agentId) const {
    return operator[](agentId).path;
}

void PathWrapper::removeTasksAndWPs(const std::unordered_set<int> &rmvTasksIndices) {
    wpList.remove_if([&](const Waypoint& wp){return rmvTasksIndices.contains(wp.getTaskIndex());});
    std::erase_if(satisfiedTasksIds,[&](int taskId){return rmvTasksIndices.contains(taskId);});
}
