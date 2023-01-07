#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include <boost/heap/fibonacci_heap.hpp>
#include "Assignment.hpp"
#include "Status.hpp"

// todo add comapre
using SmallHFibHeap = boost::heap::fibonacci_heap<Assignment>;
using SmallHHandles = std::vector<boost::heap::fibonacci_heap<Assignment>::handle_type>;

class SmallH {
public:
    SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status);

    std::pair<int, Path> extractTopAndReset();
    [[nodiscard]] TimeStep getTopMCA() const;

    void updateTopElements(const Assignment &a, const Status &status);

    void addTaskToAgent(int k, int otherTaskId, const Status &status);

    int getTaskId() const;

private:
    SmallHHandles heapHandles;
    SmallHFibHeap heap;
    int taskId;
    int v;

    static std::pair<SmallHFibHeap, std::vector<boost::heap::fibonacci_heap<Assignment>::handle_type>>
    initializeHeap(const std::vector<AgentInfo> &agentsInfos, int taskId, const Status &status);
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP