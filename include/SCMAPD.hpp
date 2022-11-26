#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include "PartialAssignment.hpp"
#include "PBS.h"
#include "Assignment.hpp"

using PASmartPtr = std::unique_ptr<PartialAssignment>;

// order each assignment by ttd
auto comparePartialAssignment = [](const PASmartPtr & a, const PASmartPtr & b){
    return *a < *b;
};

// taskIndex, robot
using PartialAssignments = std::pair<unsigned, std::vector<PASmartPtr>>;

auto compareTotalHeap = [](const PartialAssignments & a, const PartialAssignments & b){
    return *(a.second[0]) < *(b.second[0]);
};

// heap of assignments that differ by tasks
using TotalHeap = std::list<PartialAssignments>;

class SCMAPD {
public:
    SCMAPD(
            DistanceMatrix && loadedDistanceMatrix,
            std::vector<Assignment> &&robots,
            TasksVector && tasksVector
    );

    void solve(Heuristic heuristic, TimeStep cutOffTime);
private:
    const DistanceMatrix distanceMatrix;
    std::vector<Assignment> assignments;
    TasksVector tasks;
    std::unordered_set<unsigned int> unassignedTasksIndices;
    TotalHeap totalHeap;

    static TotalHeap
    buildPartialAssignmentHeap(const std::vector<Assignment> &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);

    static PASmartPtr
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Assignment &robot,
                                const TasksVector &taskVector);

    PASmartPtr extractTop();

    void updatePAsHeapTop(vector<PASmartPtr>& partialAssignments);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
