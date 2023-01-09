#include <algorithm>
#include "SmallH.hpp"

SmallH::SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status) :
        taskId{taskId},
        v{v}
    {
        std::tie(heap, heapHandles) = initializeHeap(agentsInfos, taskId, status);
    }

std::pair<SmallHFibHeap, SmallHHandles>
SmallH::initializeHeap(const std::vector<AgentInfo> &agentsInfos, int taskId, const Status &status) {
    SmallHFibHeap heap{};
    SmallHHandles handles(agentsInfos.size());

    for (const auto& aInfo : agentsInfos){
        auto agentIndex = aInfo.index;
        assert(agentIndex >= 0 && agentIndex < handles.size());
        handles[agentIndex] = heap.emplace(aInfo, taskId, status);
    }

    return {heap, handles};
}

PathWrapper SmallH::extractTopAndReset() {
    assert(!heap.empty());

    // atomic block
    auto topAssignment = std::move(const_cast<Assignment&>(heap.top()));
    heap.clear();

    return topAssignment.extractAndReset();
}

void SmallH::updateTopElements(const Path &fixedPath, const Status &status) {
    // todo check this
    for (int i = 0 ; i < std::min(v, static_cast<int>(heap.size())) ; ++i) {
        auto targetIt = std::next(heap.begin(), i);

        if(status.checkPathConflicts(fixedPath, targetIt->getPath(), false)){
            auto handle = heapHandles[targetIt->getIndex()];

            // todo check if it is possible to use increase or decrease
            // atomic
            (*handle).internalUpdate(status);
            heap.update(handle);

            // restart
            i = 0;
        }
    }
}

TimeStep SmallH::getTopMCA() const{
    assert(!heap.empty());
    return heap.top().getMCA();
}

void SmallH::addTaskToAgent(int k, int otherTaskId, const Status &status) {
    auto& targetHandle = heapHandles[k];
    assert((*targetHandle).getIndex() == k);

    //atomic
    (*targetHandle).addTask(otherTaskId, status);

    // todo check if can use increase
    heap.update(targetHandle);
}

int SmallH::getTaskId() const {
    return taskId;
}
