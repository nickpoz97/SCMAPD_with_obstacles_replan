#ifndef SIMULTANEOUS_CMAPD_BIGH_HPP
#define SIMULTANEOUS_CMAPD_BIGH_HPP

#include <vector>
#include "SmallH.hpp"

// todo check if heap is max or min
using SmallHComp = std::function<bool(const SmallH&,const SmallH&)>;
using BigHHeap = boost::heap::fibonacci_heap<SmallH, boost::heap::compare<SmallHComp>>;
using BigHHandles = std::unordered_map<int, BigHHeap::handle_type>;

class BigH {
public:
    // todo remove vector ds dependency
    BigH(const std::vector<AgentInfo> &agentInfos, const Status &status, Heuristic h, const std::vector<int> &tasksIds);

    ExtractedPath extractTop();
    [[nodiscard]] bool empty() const;

    [[nodiscard]] bool update(int k, int taskId, const Status &status);

    void addNewTasks(const std::vector<AgentInfo> &agentInfos, const Status &status,
                     const std::unordered_set<int> &newTaskIndices);

    std::vector<std::vector<std::pair<TimeStep, Assignment>>> getOrderedVector() const;

    int getNHandledTasks() const;
    void clear();
private:
    int v;
    Heuristic heuristic;

    BigHHeap heap;
    BigHHandles heapHandles;

    static SmallHComp getComparator(Heuristic h);

    bool checkOrder() const;

    bool checkIntegrity() const;
};


#endif //SIMULTANEOUS_CMAPD_BIGH_HPP
