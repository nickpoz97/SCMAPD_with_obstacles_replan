#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include <boost/heap/fibonacci_heap.hpp>
#include "Assignment.hpp"
#include "Status.hpp"

// todo add comapre
using SmallHFibHeap = boost::heap::fibonacci_heap<Assignment>;

class SmallH {
public:
    SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status);

    std::pair<int, Assignment> extractTopAndDestroy();
    [[nodiscard]] TimeStep getTopMCA() const;

    void updateTopElements(const Assignment &a, const Status &status);
    Assignment& find(int id);

    void addTaskToAgent(int k, int otherTaskId, const Status &status);

private:
    SmallHFibHeap heap;
    int taskId;
    int v;

    static SmallHFibHeap
    initializePASet(const std::vector<AgentInfo> &agentsInfos, int taskId, const Status &status);
    void sortVTop();
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP
