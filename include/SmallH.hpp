#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include "Assignment.hpp"
#include "Status.hpp"

class SmallH {
public:
    SmallH(const Status &status, const Task &task, int v);
    std::pair<int, Assignment> extractTopAndDestroy();
    [[nodiscard]] TimeStep getTopMCA() const;

    void updateTopElements(const Assignment &a, const Status &status);
    Assignment& find(int id);
private:
    std::vector<Assignment> paVec;
    const int taskId;
    const int v;

    static std::vector<Assignment> initializePASet(const Status &status, int taskId, int v);
    void sortVTop();
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP
