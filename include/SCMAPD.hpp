#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include "Robot.hpp"
#include "PBS.h"

// taskId, robot
using PartialAssignmentsVector = std::pair<unsigned, std::unique_ptr<RobotsVector>>;

// order each assignments by ttd
struct ComparePartialAssignment{
    bool operator()(const Robot & a, const Robot & b);
};

// order by the assignments wrt the task having lower ttd
struct CompareTotalHeap{
    bool operator()(const PartialAssignmentsVector & a, const PartialAssignmentsVector & b);
};

// heap of assignments that differ by tasks
using TotalHeap = std::priority_queue<PartialAssignmentsVector, std::vector<PartialAssignmentsVector>, CompareTotalHeap>;

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

class SCMAPD {
public:
    SCMAPD(
            DistanceMatrix && loadedDistanceMatrix,
            RobotsVector &&robots,
            TasksVector && tasksVector
    );

    template<Heuristic heuristic> void solve(TimeStep cutOffTime);
private:
    const DistanceMatrix distanceMatrix;
    RobotsVector assignments;
    TasksVector tasks;
    std::unordered_set<unsigned int> unassignedTasksIndices;
    TotalHeap totalHeap;

    template<Heuristic heuristic> Waypoints insert(const Task &task, const Waypoints &waypoints);

    static TotalHeap
    buildPartialAssignmentHeap(const RobotsVector &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);

    static Robot
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Robot &robot);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
