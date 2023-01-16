#include <functional>
#include <algorithm>
#include "BigH.hpp"

SmallHComp BigH::getComparator(Heuristic h) {
    switch(h){
        case Heuristic::RMCA_A:
            return [](const SmallH& a, const SmallH& b) -> bool {
                        auto aVal = a.getTopMCA() - a.getTopMCA();
                        auto bVal = b.getTopMCA() - b.getTopMCA();

                        return aVal < bVal;
                    };
        case Heuristic::RMCA_R:
            return [](const SmallH& a, const SmallH& b) -> bool {
                auto aVal = a.getTopMCA() / a.getTopMCA();
                auto bVal = b.getTopMCA() / b.getTopMCA();

                return aVal < bVal;
            };
        // MCA
        default:
            return [](const SmallH& a, const SmallH& b) -> bool {return a.getTopMCA() > b.getTopMCA();};
    }
}

BigH::BigH(const std::vector<AgentInfo> &agentInfos, const Status &status, Heuristic h) :
    v{h == Heuristic::MCA ? 1 : 2},
    heap{buildPartialAssignmentHeap(agentInfos, status, v, h)},
    heapHandles{getHandles(heap)},
    unassignedTaskIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(status.getTasks().size()))
    {
        #ifndef NDEBUG
        for(int i = 0 ; i < heapHandles.size() ; ++i){
            assert((*heapHandles[i]).getTaskId() == i);
        }
        #endif
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
    const auto& fixedPath = status.getPaths()[k];

    for(int validHandleId : unassignedTaskIndices){
        auto& sHHandle = heapHandles[validHandleId];

        // atomic
        (*sHHandle).addTaskToAgent(k, taskId, status);
        (*sHHandle).updateTopElements(fixedPath, status);
        heap.update(sHHandle);
    }
}

BigHFibHeap
BigH::buildPartialAssignmentHeap(const std::vector<AgentInfo> &agentsInfos, const Status &status, int v, Heuristic h) {
    const auto& tasks = status.getTasks();

    BigHFibHeap heap(getComparator(h));

    for(const auto& task : status.getTasks()){
        auto taskId = task.index;
        assert(taskId >= 0 && taskId < status.getTasks().size());
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
