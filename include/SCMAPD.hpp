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

class SCMAPD {
public:
    SCMAPD(
            DistanceMatrix && loadedDistanceMatrix,
            std::vector<Robot> &&robots,
            TasksVector && tasksVector
    );

    template<Heuristic heuristic> void solve(TimeStep cutOffTime);
private:
    const DistanceMatrix distanceMatrix;
    std::vector<Robot> assignments;
    TasksVector tasks;
    std::unordered_set<unsigned int> unassignedTasksIndices;
    TotalHeap totalHeap;

    static TotalHeap
    buildPartialAssignmentHeap(const std::vector<Robot> &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);

    static RobotSmartPtr
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Robot &robot);

    RobotSmartPtr extractTop();

    void updatePAsHeapTop(vector<RobotSmartPtr>& partialAssignments);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
