#include <functional>
#include "BigH.hpp"

void BigH::insert(SmallH&& smallH){
    smallHVec.insert(std::move(smallH));
}

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
    smallHVec{buildPartialAssignmentHeap(status, h, v)}
    {}

std::pair<int, Assignment> BigH::extractTopTop() {
    SmallH topSH = std::move(smallHVec.extract(smallHVec.begin()).value());
    return topSH.extractTopAndDestroy();
}

bool BigH::empty() const {
    return smallHVec.empty();
}

void BigH::updateSmallHTop(int k, const Status &status) {
    const auto& fixedAgent = status.getAssignment(k);

    for(auto it = smallHVec.begin() ; it != smallHVec.end() ;){
        auto nextIt = std::next(it);

        auto candidate = smallHVec.extract(it);
        candidate.value().updateTopElements(fixedAgent, status);
        smallHVec.insert(std::move(candidate));

        it = nextIt;
    }
}

std::vector<SmallH>
BigH::buildPartialAssignmentHeap(const Status &status, Heuristic heuristic, int v) {
    const auto& tasks = status.getTasks();

    decltype(BigH::smallHVec) bigH{};
    bigH.reserve(tasks.size());

    for(const auto& t : status.getTasks()){
        SmallH smallH(status, t, v);
        bigH.push_back(std::move(smallH));
    }
    return bigH;
}
