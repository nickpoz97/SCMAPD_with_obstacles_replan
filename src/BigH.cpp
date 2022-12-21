#include <functional>
#include <algorithm>
#include "BigH.hpp"

SmallHComp BigH::getComparator(Heuristic h) {
    switch(h){
        case Heuristic::MCA:
            return [](const SmallH& a, const SmallH& b) -> bool {return a.getTopMCA() < b.getTopMCA();};
        case Heuristic::RMCA_A:
            return [](const SmallH& a, const SmallH& b) -> bool {
                        auto aVal = a.getTopMCA() - a.getTopMCA();
                        auto bVal = b.getTopMCA() - b.getTopMCA();

                        return aVal > bVal;
                    };
        case Heuristic::RMCA_R:
            return [](const SmallH& a, const SmallH& b) -> bool {
                auto aVal = a.getTopMCA() / a.getTopMCA();
                auto bVal = b.getTopMCA() / b.getTopMCA();

                return aVal > bVal;
            };
    }
}

BigH::BigH(const Status &status, Heuristic h) :
    v{h == Heuristic::MCA ? 1 : 2},
    comparator{getComparator(h)},
    smallHVec{buildPartialAssignmentHeap(status, v)}
    {
        restoreHeapTop();
    }

std::pair<int, Assignment> BigH::extractAndDestroy() {
    assert(!smallHVec.empty());
    return smallHVec.begin()->extractTopAndDestroy();
}

bool BigH::empty() const {
    return smallHVec.empty();
}

void BigH::updateSmallHTop(int k, const Status &status) {
    const auto& fixedAgent = status.getAssignment(k);

    for(auto& sH : smallHVec){
        sH.updateTopElements(fixedAgent, status);
    }
    restoreHeapTop();
}

std::vector<SmallH>
BigH::buildPartialAssignmentHeap(const Status &status, int v) {
    const auto& tasks = status.getTasks();

    decltype(BigH::smallHVec) bigH{};
    bigH.reserve(tasks.size());

    for(const auto& t : status.getTasks()){
        SmallH smallH(status, t, v);
        bigH.push_back(std::move(smallH));
    }
    return bigH;
}

void BigH::restoreHeapTop() {
    assert(!smallHVec.empty());
    auto firstElIt = smallHVec.begin();
    auto minElIt = std::min_element(smallHVec.begin(), smallHVec.end(), comparator);
    std::iter_swap(firstElIt, minElIt);
}

void BigH::updateOtherPAs(int k, Status status, int taskId) {
    for(auto& sH : smallHVec){
        sH.find(k).addTask(status.getAmbientMapInstance(), status.getConstraints(), taskId, status.getTasks());
    }

    restoreHeapTop();
}
