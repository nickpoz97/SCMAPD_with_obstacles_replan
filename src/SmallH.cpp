#include <algorithm>
#include "SmallH.hpp"
#include "NotPossibleOptimization.hpp"
#include "MAPF/NoPathException.hpp"

SmallH::SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status) :
        taskId{taskId},
        v{v},
        heap{},
        heapHandles{},
        removedAgents{}
{
    heapHandles.reserve(agentsInfos.size());

    for (const auto& aInfo : agentsInfos){
        auto agentIndex = aInfo.index;
        try{
            heapHandles.emplace(agentIndex, heap.emplace(aInfo, taskId, status));
        }
        catch(const NoPathException& e) {
            removedAgents.insert(agentIndex);
        }
        assert(agentIndex >= 0 && agentIndex < agentsInfos.size());
    }

    #ifndef NDEBUG
    for(int i = 0 ; i < heapHandles.size() ; ++i){
        assert((*heapHandles[i]).getAgentId() == i);
    }
    #endif
    assert(checkOrder());
}

SmallH::SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status,
               const PWsVector &pWs) :
    taskId{taskId},
    v{v},
    heap{},
    heapHandles{},
    removedAgents{}
{
    heapHandles.reserve(agentsInfos.size());

    assert(agentsInfos.size() == pWs.size());

    for (int agentIndex = 0 ; agentIndex < pWs.size() ; ++agentIndex){
        const auto& pW = pWs[agentIndex];
        const auto& aInfo = agentsInfos[agentIndex];
        assert(aInfo.index == agentIndex);

        try{
            heapHandles.emplace(agentIndex, heap.emplace(aInfo, taskId, status, pW));
        }
        catch(const NoPathException& e) {
            removedAgents.insert(agentIndex);
        }
    }
    if(heap.empty()){
        throw NotPossibleOptimization();
    }
}

std::tuple<int, TimeStep, Path> SmallH::extractTop() {
    assert(!heap.empty());

    // atomic block
    auto topAssignment = std::move(const_cast<Assignment&>(heap.top()));

    return topAssignment.extractAndReset();
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

            try{
                (*handle).internalUpdate(status);
            }
            catch(const NoPathException& e){
                heap.erase(handle);
                removedAgents.insert(agentId);
                continue;
            }
            heap.update(handle);
        }
    }
    assert(!status.checkPathWithStatus(getTopPath(), getTopAgentId()));
    assert(checkOrder());
}

TimeStep SmallH::getTopMCA() const{
    assert(!heap.empty());
    return heap.top().getMCA();
}

void SmallH::addTaskToAgent(int k, int otherTaskId, const Status &status) {
    assert(checkOrder());

    if(removedAgents.contains(k)){
        return;
    }

    auto& targetHandle = heapHandles[k];
    assert((*targetHandle).getAgentId() == k);

    try{
        (*targetHandle).addTask(otherTaskId, status);
    }
    catch(const NoPathException& e){
        heap.erase(targetHandle);
        removedAgents.insert(k);
        return;
    }
    heap.update(targetHandle);
    assert(checkOrder());
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
    return heap.empty();
}
[[nodiscard]] ExtractedPath SmallH::getTopWrappedPath() const{
    const auto& top = heap.top();

    return{
        .newTaskId = taskId,
        .agentId = top.getAgentId(),
        .wrapper{
            {top.getPath()},
            {top.getWaypoints()},
            {top.getAssignedTaskIds()}
        }
    };
}

const Assignment &SmallH::getSecondTopAssignment() const {
    assert(heap.size() >= 2);
    return *std::next(heap.ordered_begin(), 2);
}

TimeStep SmallH::getSecondTopMCA() const {
    return getSecondTopAssignment().getMCA();
}

