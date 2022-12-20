#ifndef SIMULTANEOUS_CMAPD_BIGH_HPP
#define SIMULTANEOUS_CMAPD_BIGH_HPP

#include <vector>
#include "SmallH.hpp"

using SmallHComp = std::function<bool(const SmallH&,const SmallH&)>;

class BigH {
public:
    explicit BigH(const Status &status, Heuristic h);
    void insert(SmallH&& smallH);
    std::pair<int, Assignment> extractTopTop();
    [[nodiscard]] bool empty() const;
    void updateSmallHTop(int k, const Status &status);
private:
    int v;
    SmallHComp comparator;
    std::vector<SmallH> smallHVec;

    static SmallHComp getComparator(Heuristic h);
    static std::vector<SmallH> buildPartialAssignmentHeap(const Status &status, Heuristic heuristic, int v);
};


#endif //SIMULTANEOUS_CMAPD_BIGH_HPP
