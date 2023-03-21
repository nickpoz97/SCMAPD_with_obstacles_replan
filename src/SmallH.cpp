#include <algorithm>
#include "SmallH.hpp"

SmallH::SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status,
               const PWsVector &pWs) :
    taskId{taskId},
    v{v},
    heap{},
    heapHandles{}
{
    assert(pWs.empty() || pWs.size() == agentsInfos.size());

    heapHandles.reserve(agentsInfos.size());

    for (const auto& aInfo : agentsInfos){
        auto agentIndex = aInfo.index;

        auto handle = heap.emplace(aInfo);

        if(!pWs.empty()){
            (*handle) = pWs[agentIndex];
        }

        if(!(*handle).addTask(taskId, status)){
            heap.erase(handle);
            assert(heap.size() == heapHandles.size());
            continue;
        }
        heap.update(handle);
        heapHandles.emplace(agentIndex, handle);
    }

    assert(heap.size() == heapHandles.size());
#ifndef NDEBUG
    for(const auto& aInfo : agentsInfos){
        auto i = aInfo.index;
        assert(!heapHandles.contains(i) || (*heapHandles[i]).getAgentId() == i);
    }
#endif
    assert(checkOrder());
}

void SmallH::updateTopElements(const Status &status) {
    assert(checkOrder());

    // todo check this
    for (int i = 0 ; i < std::min(v, static_cast<int>(heap.size())) ; ++i) {
        auto targetIt = std::next(heap.ordered_begin(), i);

        auto agentId = targetIt->getAgentId();
        if(status.checkPathWithStatus(targetIt->getPath(), targetIt->getAgentId())){
            // restart index
            i = -1;

            auto& handle = heapHandles[agentId];
            assert((*handle).getAgentId() == agentId);

            if(!(*handle).internalUpdate(status, true)){
                heap.erase(handle);
                heapHandles.erase(agentId);
                assert(heap.size() == heapHandles.size());
                continue;
            }
            heap.update(handle);
        }
    }
    assert(!status.checkPathWithStatus(getTopPath(), getTopAgentId()));
    assert(checkOrder());
}

void SmallH::addTaskToAgent(int k, int otherTaskId, const Status &status) {
    assert(checkOrder());

    if(!heapHandles.contains(k)){
        return;
    }

    auto& targetHandle = heapHandles[k];
    assert((*targetHandle).getAgentId() == k);

    if(!(*targetHandle).addTask(otherTaskId, status)){
        heap.erase(targetHandle);
        heapHandles.erase(k);
        assert(heap.size() == heapHandles.size());
        return;
    }
    heap.update(targetHandle);
    assert(checkOrder());
}

int SmallH::getTaskId() const {
    return taskId;
}

const Path &SmallH::getTopPath() const {
    return heap.top().getPath();
}
int SmallH::getTopAgentId() const{
    return heap.top().getAgentId();
}

const Assignment &SmallH::getTopAssignment() const {
    assert(!heap.empty());
    return heap.top();
}

bool SmallH::checkOrder() const{
    return std::is_sorted(heap.ordered_begin(), heap.ordered_end());
}

std::vector<std::pair<TimeStep, Assignment>> SmallH::getOrderedVector() const{
    std::vector<std::pair<TimeStep, Assignment>> vec;
    vec.reserve(heap.size());

    for(auto it = heap.ordered_begin(); it != heap.ordered_end() ; ++it){
        vec.emplace_back(it->getMCA(), *it);
    }
    return vec;
}

bool SmallH::empty() const{
    assert(heap.size() == heapHandles.size());
    return heap.empty();
}
[[nodiscard]] ExtractedPath SmallH::getTopWrappedPath() const{
    const auto& top = heap.top();

    return{
        .newTaskId = taskId,
        .agentId = top.getAgentId(),
        .wrapper{top}
    };
}

const Assignment &SmallH::getSecondTopAssignment() const {
    assert(heap.size() >= 2);
    return *std::next(heap.ordered_begin(), 2);
}
