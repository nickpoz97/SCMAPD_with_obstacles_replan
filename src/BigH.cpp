#include <functional>
#include <algorithm>
#include "BigH.hpp"

SmallHComp BigH::getComparator(Heuristic h) {
    switch(h){
        // todo not tested
        case Heuristic::RMCA_A:
            return [](const SmallH& a, const SmallH& b) -> bool {
                auto aVal = a.getTopMCA() - a.getTopMCA();
                auto bVal = b.getTopMCA() - b.getTopMCA();

                return aVal < bVal;
            };
        // todo not tested
        case Heuristic::RMCA_R:
            return [](const SmallH& a, const SmallH& b) -> bool {
                auto aVal = a.getTopMCA() / a.getTopMCA();
                auto bVal = b.getTopMCA() / b.getTopMCA();

                return aVal < bVal;
            };
        // MCA
        default:
            return [](const SmallH& a, const SmallH& b) -> bool {return a.getTopAssignment() > b.getTopAssignment();};
    }
}

BigH::BigH(const std::vector<AgentInfo> &agentInfos, const Status &status, Heuristic h) :
    v{h == Heuristic::MCA ? 1 : 2},
    heuristic{h},
    heap{buildPartialAssignmentHeap(agentInfos, status, v, h)},
    heapHandles{getHandles(heap)},
    unassignedTaskIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(status.getTasks().size()))
    {
        #ifndef NDEBUG
        for(int i = 0 ; i < heapHandles.size() ; ++i){
            assert((*heapHandles[i]).getTaskId() == i);
        }
        #endif
        assert(checkOrder());
    }

ExtractedPath BigH::extractTop() {
    assert(!heap.empty());

    // atomic block (and order is important)
    auto topSmallH = std::move(const_cast<SmallH&>(heap.top()));
    auto taskId = topSmallH.getTaskId();
    auto pathWrapper = topSmallH.extractTopAndReset();
    heap.pop();

    unassignedTaskIndices.erase(taskId);
    return {taskId, std::move(pathWrapper)};
}

bool BigH::empty() const {
    return heap.empty();
}

void BigH::update(int k, int taskId, const Status &status) {
    assert(checkOrder());
    for(auto otherTaskId : unassignedTaskIndices){
        auto& sHHandle = heapHandles[otherTaskId];
        if((*sHHandle).empty()){
            throw std::runtime_error("No way to find all paths");
        }

        assert((*sHHandle).getTaskId() == otherTaskId);

        // atomic
        (*sHHandle).addTaskToAgent(k, taskId, status);
        (*sHHandle).updateTopElements(status);
        heap.update(sHHandle);

        assert((*sHHandle).getTaskId() == otherTaskId);
        assert(!status.checkPathWithStatus((*sHHandle).getTopPath(), (*sHHandle).getTopAgentId()));
        assert(checkOrder());
    }
#ifndef NDEBUG
    for(const auto& sH : heap){
        assert(!status.checkPathWithStatus(sH.getTopPath(), sH.getTopAgentId()));
    }
#endif
}

BigHFibHeap
BigH::buildPartialAssignmentHeap(const std::vector<AgentInfo> &agentsInfos, const Status &status, int v, Heuristic h) {
    const auto& tasks = status.getTasks();

    BigHFibHeap heap(getComparator(h));

    for(const auto& task : tasks){
        auto taskId = task.index;
        assert(taskId >= 0 && taskId < tasks.size());
        heap.emplace(agentsInfos, taskId, v, status);
    }

    return heap;
}

BigHHandles BigH::getHandles(const BigHFibHeap &heap) {
    BigHHandles heapHandles{};

    for(auto it = heap.begin(); it != heap.end() ; ++it){
        heapHandles.emplace(it->getTaskId(), BigHFibHeap::s_handle_from_iterator(it));
    }
    return heapHandles;
}

bool BigH::checkOrder() const{
    auto compare = [](const SmallH& a, const SmallH& b){
        return a.getTopAssignment() < b.getTopAssignment();
    };

    return std::is_sorted(heap.ordered_begin(), heap.ordered_end(), compare);
}

std::vector<std::vector<std::pair<TimeStep, Assignment>>> BigH::getReverseOrderedVector() const{
    std::vector<std::vector<std::pair<TimeStep, Assignment>>> vec;
    vec.reserve(heap.size());

    for(auto it = heap.ordered_begin() ; it != heap.ordered_end() ; ++it){
        vec.push_back(it->getOrderedVector());
    }

    return vec;
}

