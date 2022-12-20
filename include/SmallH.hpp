#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include <set>
#include "Assignment.hpp"
#include "Status.hpp"

class SmallH {
public:
    SmallH(const std::vector<Assignment> &agents, const Task &task, const cmapd::AmbientMapInstance &instance);
    std::pair<int, Assignment> extractTop();
    [[nodiscard]] TimeStep getTopMCA() const;

    void updateSmallHTop(const Assignment &a, int v, const Status& status);
private:
    std::set<Assignment> paSet;
    int taskId;

    static std::set<Assignment> initializePASet(const std::vector<Assignment> &agents, const Task &task,
                                                const cmapd::AmbientMapInstance &instance);
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP
