#ifndef SIMULTANEOUS_CMAPD_BIGH_HPP
#define SIMULTANEOUS_CMAPD_BIGH_HPP

#include <vector>
#include "SmallH.hpp"

// todo check if heap is max or min
using SmallHComp = std::function<bool(const SmallH&,const SmallH&)>;
using BigHFibHeap = boost::heap::binomial_heap<SmallH, boost::heap::compare<SmallHComp>>;
using BigHHandles = std::unordered_map<int, BigHFibHeap::handle_type>;

class BigH {
public:
    BigH(const std::vector<AgentInfo> &agentInfos, const Status &status, Heuristic h);

    ExtractedPath extractTop();
    [[nodiscard]] bool empty() const;

    void update(int k, int taskId, const Status &status);

    void addNewTasks(const std::vector<AgentInfo> &agentInfos, const Status &status,
                     std::unordered_set<int> &&newTaskIndices);

    std::vector<std::vector<std::pair<TimeStep, Assignment>>> getOrderedVector() const;

private:
    int v;
    Heuristic heuristic;

    BigHFibHeap heap;
    BigHHandles heapHandles;

    std::unordered_set<int> unassignedTaskIndices;
    static SmallHComp getComparator(Heuristic h);

    static BigHFibHeap
    buildPartialAssignmentHeap(
        const std::vector<AgentInfo> &agentsInfos, const Status &status, int v, Heuristic h
    );
    static BigHHandles getHandles(const BigHFibHeap& heap);

    bool checkOrder() const;
};


#endif //SIMULTANEOUS_CMAPD_BIGH_HPP
