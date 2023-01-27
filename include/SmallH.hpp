#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include <boost/heap/fibonacci_heap.hpp>
#include "Assignment.hpp"
#include "Status.hpp"
#include "AgentInfo.hpp"

using SmallHFibHeap = boost::heap::fibonacci_heap<Assignment, boost::heap::compare<std::greater<>>>;
using SmallHHandles = std::unordered_map<int, SmallHFibHeap::handle_type>;

class SmallH {
public:
    SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status);

    PathWrapper extractTopAndReset();
    [[nodiscard]] TimeStep getTopMCA() const;

    void updateTopElements(int agentId, const Status &status);
    void addTaskToAgent(int k, int otherTaskId, const Status &status);
    int getTaskId() const;

    const Path& getTopPath() const;
    int getTopAgentId() const;

    const Assignment& getTopAssignment() const;
private:
    int taskId;
    int v;
    SmallHFibHeap heap;
    SmallHHandles heapHandles;

    static SmallHFibHeap
    initializeHeap(const std::vector<AgentInfo> &agentsInfos, int taskId, const Status &status);

    static SmallHHandles getHandles(const SmallHFibHeap& heap);
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP
