#include <functional>
#include <algorithm>
#include "BigH.hpp"

SmallHComp BigH::getComparator(Heuristic h) {
    switch(h){
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
        case Heuristic::RMCA_R:

            return [](const SmallH& a, const SmallH& b) -> bool {
                auto getVal = [](const SmallH& sH){
                    auto fMCA = sH.getTopAssignment().getMCA();
                    auto sMCA = sH.getSecondTopAssignment().getMCA();

                    fMCA += fMCA >= 0 ? 1 : -1;
                    sMCA += sMCA >= 0 ? 1 : -1;

                    return static_cast<float>(sMCA) / static_cast<float>(fMCA);
                };

                auto aVal = getVal(a);
                auto bVal = getVal(b);

                return aVal < bVal || (aVal == bVal && a.getTopAssignment() > b.getTopAssignment());
            };
        // MCA
        default:
            return [](const SmallH& a, const SmallH& b) -> bool {
                assert(!a.empty() && !b.empty());
                return a.getTopAssignment() > b.getTopAssignment();
            };
    }
}

BigH::BigH(Heuristic h) :
    v{h == Heuristic::MCA ? 1 : 2},
    heuristic{h},
    heap(getComparator(h))
    {}

ExtractedPath BigH::extractTop() {
    assert(!heap.empty());

    auto extractedPath = heap.top().getTopWrappedPath();

    heap.pop();
    heapHandles.erase(extractedPath.newTaskId);

    assert(checkIntegrity());
    return extractedPath;
}

bool BigH::empty() const {
    assert(checkIntegrity());
    return heap.empty();
}

bool BigH::update(int k, int taskId, const Status &status) {

    assert(checkOrder());
    for(auto& [otherTaskId, sHHandle] : heapHandles){
        assert((*sHHandle).getTaskId() == otherTaskId);

        // atomic
        (*sHHandle).addTaskToAgent(k, taskId, status);
        heap.update(sHHandle);
        (*sHHandle).updateTopElements(status);
        heap.update(sHHandle);

        assert(checkIntegrity());
        if((*sHHandle).empty()){
            // impossible to find path
            return false;
        }
        heap.update(sHHandle);

        assert(checkIntegrity());
        assert((*sHHandle).getTaskId() == otherTaskId);
        assert(!status.checkPathWithStatus((*sHHandle).getTopPath(), (*sHHandle).getTopAgentId()));
        assert(checkOrder());
    }
#ifndef NDEBUG
    for(const auto& sH : heap){
        assert(!status.checkPathWithStatus(sH.getTopPath(), sH.getTopAgentId()));
    }
#endif
    return true;
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

void BigH::addNewTasks(const Status &status, const std::vector<int> &newTaskIndices, const std::vector<int> &availableAgentIds) {
    for(int taskId : newTaskIndices){
        // if tha UTI contains it this mean you re-added a task
        // this exploits the fact we should not have index value overflow
        assert(!heapHandles.contains(taskId));
        assert(status.taskIdExists(taskId));
        heapHandles.emplace(taskId, heap.emplace(taskId, v, status, availableAgentIds));
    }

    assert(checkIntegrity());
    assert(checkOrder());
}

bool BigH::checkIntegrity() const{
    return heap.size() == heapHandles.size() &&
        std::ranges::all_of(heap, [this](const SmallH& sH){return heapHandles.contains(sH.getTaskId());});
}

void BigH::clear() {
    heap.clear();
    heapHandles.clear();
}
