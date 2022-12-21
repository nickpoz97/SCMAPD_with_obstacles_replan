#ifndef SIMULTANEOUS_CMAPD_BIGH_HPP
#define SIMULTANEOUS_CMAPD_BIGH_HPP

#include <vector>
#include "SmallH.hpp"

using SmallHComp = std::function<bool(const SmallH&,const SmallH&)>;

class BigH {
public:
    BigH(const Status &status, Heuristic h);
    std::pair<int, Assignment> extractAndDestroy();
    [[nodiscard]] bool empty() const;

    void update(int k, int taskId, const Status &status);

private:
    int v;
    SmallHComp comparator;
    std::vector<SmallH> smallHVec;

    static SmallHComp getComparator(Heuristic h);
    static std::vector<SmallH> buildPartialAssignmentHeap(const Status &status, int v);
    void restoreHeapTop();
};


#endif //SIMULTANEOUS_CMAPD_BIGH_HPP
