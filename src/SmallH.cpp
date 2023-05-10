#include <algorithm>
#include "SmallH.hpp"

SmallH::SmallH(int taskId, int v, const Status &status) :
    taskId{taskId},
    v{v},
    heap{},
    heapHandles{}
{
    const auto& pWs = status.getPathWrappers();

    for (const auto& pW : status.getPathWrappers()){
        Assignment partialAssignment{pW, const_cast<Status&>(status)};

        if(!partialAssignment.addTask(taskId)){
            continue;
        }

        heapHandles.emplace(partialAssignment.getAgentId(), heap.push(partialAssignment));
    }

    assert(heap.size() == heapHandles.size());
    #ifndef NDEBUG
        for(const auto& pW : pWs){
            auto i = pW.getAgentId();
            assert(!heapHandles.contains(i) || (*heapHandles[i]).getAgentId() == i);
        }
    #endif
    assert(checkOrder());
}

void SmallH::updateTopElements(const Status &status) {
    assert(checkOrder());

    for (int i = 0 ; i < std::min(v, static_cast<int>(heap.size())) ; ++i) {
        auto targetIt = std::next(heap.ordered_begin(), i);

        auto agentId = targetIt->getAgentId();
        if(status.checkPathWithStatus(targetIt->getPath(), targetIt->getAgentId())){
            // restart index
            i = -1;

            auto& handle = heapHandles[agentId];
            assert((*handle).getAgentId() == agentId);

            if(!(*handle).internalUpdate()){
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

void SmallH::addTaskToAgent(int k, int otherTaskId) {
    assert(checkOrder());

    if(!heapHandles.contains(k)){
        return;
    }

    auto& targetHandle = heapHandles[k];
    assert((*targetHandle).getAgentId() == k);

    if(!(*targetHandle).addTask(otherTaskId)){
        heap.erase(targetHandle);
        heapHandles.erase(k);
        assert(heap.size() == heapHandles.size());
        assert(checkOrder());
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
    return *std::next(heap.ordered_begin());
}

bool SmallH::hasOnlyOneAssignment() const {
    return heap.size() == 1;
}
