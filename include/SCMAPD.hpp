#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include "Robot.hpp"
#include "PBS.h"

using RobotSmartPtr = std::unique_ptr<Robot>;

// order each assignment by ttd
auto comparePartialAssignment = [](const RobotSmartPtr & a, const RobotSmartPtr & b){
    return a->getTtd() > b->getTtd();
};

// taskId, robot
using PartialAssignments = std::pair<unsigned, std::vector<RobotSmartPtr>>;

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

    template<Heuristic heuristic> Waypoints insert(const Task &task, const Waypoints &waypoints);

    static TotalHeap
    buildPartialAssignmentHeap(const RobotsVector &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);

    static std::unique_ptr<Robot>
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Robot &robot);

    RobotSmartPtr extractTop();
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
