#include <algorithm>
#include "SmallH.hpp"

SmallH::SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status) :
        taskId{taskId},
        v{v},
        heap{initializeHeap(agentsInfos, taskId, status)},
        heapHandles{getHandles(heap)}
    {
        #ifndef NDEBUG
        for(int i = 0 ; i < heapHandles.size() ; ++i){
            assert((*heapHandles[i]).getAgentId() == i);
        }
        #endif
    }

SmallHFibHeap
SmallH::initializeHeap(const std::vector<AgentInfo> &agentsInfos, int taskId, const Status &status) {
    SmallHFibHeap heap{};

    for (const auto& aInfo : agentsInfos){
        auto agentIndex = aInfo.index;
        heap.emplace(aInfo, taskId, status);
        assert(agentIndex >= 0 && agentIndex < agentsInfos.size());
    }

    return heap;
}

PathWrapper SmallH::extractTopAndReset() {
    assert(!heap.empty());

    // atomic block
    auto topAssignment = std::move(const_cast<Assignment&>(heap.top()));
    heap.clear();

    return topAssignment.extractAndReset();
}

void SmallH::updateTopElements(int agentId, const Status &status) {
    // todo check this
    for (int i = 0 ; i < std::min(v, static_cast<int>(heap.size())) ; ++i) {
        auto targetIt = std::next(heap.ordered_begin(), i);

        if(status.checkPathWithStatus(targetIt->getPath(), agentId)){
            auto& handle = heapHandles[targetIt->getAgentId()];
            assert((*handle).getAgentId() == targetIt->getAgentId());

            // todo check if it is possible to use increase or decrease
            // atomic
            (*handle).internalUpdate(status);
            heap.update(handle);

            // restart
            i = -1;
        }
    }
    assert(!status.checkPathWithStatus(getTopPath(), getTopAgentId()));
}

TimeStep SmallH::getTopMCA() const{
    assert(!heap.empty());
    return heap.top().getMCA();
}

void SmallH::addTaskToAgent(int k, int otherTaskId, const Status &status) {
    auto& targetHandle = heapHandles[k];
    assert((*targetHandle).getAgentId() == k);

    //atomic
    (*targetHandle).addTask(otherTaskId, status);

    // todo check if can use increase
    heap.update(targetHandle);
}

int SmallH::getTaskId() const {
    return taskId;
}

SmallHHandles SmallH::getHandles(const SmallHFibHeap& heap){
    SmallHHandles heapHandles{};

    for(auto it = heap.begin(); it != heap.end() ; ++it){
        heapHandles.emplace(it->getAgentId(), SmallHFibHeap::s_handle_from_iterator(it));
    }
    return heapHandles;
}

const Path &SmallH::getTopPath() const {
    return heap.top().getPath();
}
int SmallH::getTopAgentId() const{
    return heap.top().getAgentId();
}

const Assignment &SmallH::getTopAssignment() const {
    return heap.top();
}
