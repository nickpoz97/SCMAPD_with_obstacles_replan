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

using Assignment = std::vector<Robot>;

struct ComparePartialAssignment{
    bool operator()(const Robot& a, const Robot& b);
};

struct CompareTotalHeap{
    bool operator()(const Assignment & a, const Assignment & b);
};

using PartialAssignmentHeap = std::priority_queue<Assignment, std::vector<Assignment>, CompareTotalHeap>;

class SCMAPD {
public:
    SCMAPD(
        DistanceMatrix && distanceMatrix,
        Assignment && robots,
        TaskSet && tasks
    );

    template<Heuristic heuristic>
    void solve(TimeStep cutOffTime);
private:
    const DistanceMatrix distanceMatrix;
    Assignment assignment;
    TaskSet unassignedTasks;
    PartialAssignmentHeap partialAssignmentsHeap;

    template<Heuristic heuristic>
    Waypoints insert(const Task &task, const Waypoints &waypoints);

    static PartialAssignmentHeap
    buildPartialAssignmentHeap(const Assignment &robots, const TaskSet &tasks,
                               const DistanceMatrix &distanceMatrix);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
