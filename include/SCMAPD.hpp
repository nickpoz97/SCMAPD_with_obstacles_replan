#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include "Robot.hpp"
#include "PBS.h"

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

// a fake heap
using PartialAssignment = std::vector<Robot>;

using Assignment = std::unordered_map<CompressedCoord, Robot>;

struct ComparePartialAssignment{
    bool operator()(const Robot& a, const Robot& b);
};

struct CompareTotalHeap{
    bool operator()(const PartialAssignment & a, const PartialAssignment & b);
};

using PartialAssignmentHeap = std::priority_queue<PartialAssignment, std::vector<PartialAssignment>, CompareTotalHeap>;

class SCMAPD {
public:
    SCMAPD(
            DistanceMatrix && distanceMatrix,
            Assignment &&robots,
            TasksVector && tasks
    );

    template<Heuristic heuristic>
    void solve(TimeStep cutOffTime);
private:
    const DistanceMatrix distanceMatrix;
    Assignment assignment;
    TasksVector tasks;
    std::unordered_set<unsigned int> unassignedTasksIndices;
    PartialAssignmentHeap partialAssignmentsHeap;

    template<Heuristic heuristic>
    Waypoints insert(const Task &task, const Waypoints &waypoints);

    static PartialAssignmentHeap
    buildPartialAssignmentHeap(const Assignment &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
