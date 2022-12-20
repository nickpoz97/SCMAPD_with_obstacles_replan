#include <functional>
#include "BigH.hpp"

void BigH::insert(SmallH&& smallH){
    smallHSet.insert(std::move(smallH));
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

BigH::BigH(Heuristic h) : smallHSet(getComparator(h)){}

std::pair<int, Assignment> BigH::extractTopTop() {
    SmallH topSH = std::move(smallHSet.extract(smallHSet.begin()).value());
    return topSH.extractTop();
}
