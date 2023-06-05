#ifndef SIMULTANEOUS_CMAPD_SMALLH_HPP
#define SIMULTANEOUS_CMAPD_SMALLH_HPP

#include <vector>
#include <boost/heap/fibonacci_heap.hpp>
#include "Assignment.hpp"
#include "Status.hpp"
#include "AgentInfo.hpp"

using SmallHHeap = boost::heap::fibonacci_heap<Assignment, boost::heap::compare<std::greater<>>>;
using SmallHHandles = std::unordered_map<int, SmallHHeap::handle_type>;

class SmallH {
public:
    SmallH(int taskId, int v, const Status &status);

    void updateTopElements(const Status &status);
    void addTaskToAgent(int k, int otherTaskId);

    [[nodiscard]] const Assignment& getTopAssignment() const;
    [[nodiscard]] const Assignment& getSecondTopAssignment() const;

    std::vector<std::pair<TimeStep, Assignment>> getOrderedVector() const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] int getTaskId() const;
    [[nodiscard]] const Path& getTopPath() const;
    [[nodiscard]] int getTopAgentId() const;

    [[nodiscard]] ExtractedPath getTopWrappedPath() const;

    [[nodiscard]] bool hasOnlyOneAssignment() const;
private:
    int taskId;
    int v;
    SmallHHeap heap;
    SmallHHandles heapHandles;

    bool checkOrder() const;
};


#endif //SIMULTANEOUS_CMAPD_SMALLH_HPP