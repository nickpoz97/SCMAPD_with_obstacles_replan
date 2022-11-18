#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include "Robot.hpp"
#include "PBS.h"

#include "PartialAssignment.hpp"

template<Heuristic heuristic>
struct CompareTotalHeap{
    bool operator()(const PartialAssignment<heuristic> & a, const PartialAssignment<heuristic> & b);
};

using PartialAssignmentHeap = std::priority_queue<PartialAssignment, std::vector<PartialAssignment>, CompareTotalHeap>;

class SCMAPD {
public:
    SCMAPD(
            DistanceMatrix && loadedDistanceMatrix,
            Assignment &&robots,
            TasksVector && tasksVector
    );

    template<Heuristic heuristic> void solve(TimeStep cutOffTime);
private:
    const DistanceMatrix distanceMatrix;
    Assignment assignment;
    TasksVector tasks;
    std::unordered_set<unsigned int> unassignedTasksIndices;
    PartialAssignmentHeap partialAssignmentsHeap;

    template<Heuristic heuristic> Waypoints insert(const Task &task, const Waypoints &waypoints);

    static PartialAssignmentHeap
    buildPartialAssignmentHeap(const Assignment &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
