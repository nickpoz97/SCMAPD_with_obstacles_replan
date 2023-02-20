#include <functional>
#include <algorithm>
#include "BigH.hpp"

SmallHComp BigH::getComparator(Heuristic h) {
    switch(h){
        // todo not tested
        case Heuristic::RMCA_A:
            return [](const SmallH& a, const SmallH& b) -> bool {
                auto getVal = [](const SmallH& sH){
                    auto fMCA = sH.getTopAssignment().getMCA();
                    auto sMCA = sH.getSecondTopAssignment().getMCA();

                    return sMCA - fMCA;
                };

                auto aVal = getVal(a);
                auto bVal = getVal(b);
                assert(aVal >= 0 && bVal >= 0);

                return aVal < bVal || (aVal == bVal && a.getTopAssignment() > b.getTopAssignment());
            };
        // todo not tested
        case Heuristic::RMCA_R:

            return [](const SmallH& a, const SmallH& b) -> bool {
                auto getVal = [](const SmallH& sH){
                    auto fMCA = sH.getTopAssignment().getMCA();
                    auto sMCA = sH.getSecondTopAssignment().getMCA();

                    assert(fMCA >= 0 && sMCA >= fMCA);
                    if(fMCA == 0 && sMCA == 0){
                        return 1.0f;
                    }
                    if(fMCA == 0 && sMCA >= 0){
                        return std::numeric_limits<float>::infinity();
                    }
                    return static_cast<float>(sMCA) / static_cast<float>(fMCA);
                };

                auto aVal = getVal(a);
                auto bVal = getVal(b);

                return aVal < bVal || (aVal == bVal && a.getTopAssignment() > b.getTopAssignment());
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

    const auto& topSmallH = heap.top();
    auto extractedPath = topSmallH.getTopWrappedPath();

    heap.pop();
    unassignedTaskIndices.erase(extractedPath.newTaskId);

    return extractedPath;
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

BigHHandles BigH::getHandles(const BigHFibHeap &heap) {
    BigHHandles heapHandles{};

    for(auto it = heap.begin(); it != heap.end() ; ++it){
        heapHandles.emplace(it->getTaskId(), BigHFibHeap::s_handle_from_iterator(it));
    }
    return heapHandles;
}

bool BigH::checkOrder() const{
    const auto reversedComparator = [&](const SmallH& a, const SmallH&b) {
        return getComparator(heuristic)(b, a);
    };

    return std::is_sorted(heap.ordered_begin(), heap.ordered_end(), reversedComparator);
}

std::vector<std::vector<std::pair<TimeStep, Assignment>>> BigH::getOrderedVector() const{
    std::vector<std::vector<std::pair<TimeStep, Assignment>>> vec;
    vec.reserve(heap.size());

    for(auto it = heap.ordered_begin() ; it != heap.ordered_end() ; ++it){
        vec.push_back(it->getOrderedVector());
    }

    return vec;
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

void BigH::addNewTasks(const std::vector<AgentInfo> &agentInfos, const PWsVector &pathsWrappers, const Status &status,
                       const std::unordered_set<int> &taskIndices) {
    for (int taskId : taskIndices){
        // if tha UTI contains it this mean you re-added a task
        // this exploits the fact we should not have index value overflow
        assert(!unassignedTaskIndices.contains(taskId));

        heapHandles[taskId] = heap.emplace(agentInfos, taskId, v, status, pathsWrappers);
    }
    unassignedTaskIndices = taskIndices;
}

