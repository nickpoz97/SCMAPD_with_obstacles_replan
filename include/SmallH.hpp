#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include <set>
#include "Assignment.hpp"
#include "Status.hpp"

class SmallH {
public:
    SmallH(const Status &status, const Task &task);
    std::pair<int, Assignment> extractTop();
    [[nodiscard]] TimeStep getTopMCA() const;

    void updateSmallHTop(const Assignment &a, int v, const Status& status);
private:
    std::set<Assignment> paSet;
    int taskId;

    static std::set<Assignment> initializePASet(const Status &status, int taskId);
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP
