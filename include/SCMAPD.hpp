#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include <vector>
#include "PartialAssignment.hpp"
#include "Assignment.hpp"

// taskIndex, robot
using PartialAssignments = std::pair<unsigned, std::vector<PartialAssignment>>;

inline auto compareTotalHeap = [](const PartialAssignments & a, const PartialAssignments & b){
    return (a.second[0]) < (b.second[0]);
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

    static PartialAssignment
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Assignment &robot,
                                const TasksVector &taskVector);

    PartialAssignment extractTop();

    void updatePAsHeapTop(const std::vector<PartialAssignment> &partialAssignments);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
