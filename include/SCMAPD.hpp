#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include "Robot.hpp"
#include "PBS.h"
#include "PartialAssignment.hpp"

using PASmartPtr = std::unique_ptr<PartialAssignment>;

// order each assignment by ttd
auto comparePartialAssignment = [](const PASmartPtr & a, const PASmartPtr & b){
    return a->getTtd() > b->getTtd();
};

// taskId, robot
using PartialAssignments = std::pair<unsigned, std::vector<PASmartPtr>>;

auto compareTotalHeap = [](const PartialAssignments & a, const PartialAssignments & b){
    return a.second[0] < b.second[0];
};

// heap of assignments that differ by tasks
using TotalHeap = std::list<PartialAssignments>;

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

    // updates pa waypoints and ttd using heuristic
    template<Heuristic heuristic>
    void insert(const Task &task, const Waypoints &waypoints, PartialAssignment *partialAssignmentPtr);

    static TotalHeap
    buildPartialAssignmentHeap(const RobotsVector &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);

    static PASmartPtr
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Robot &robot);

    PASmartPtr extractTop();
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
