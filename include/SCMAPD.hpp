#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include <vector>
#include "Assignment.hpp"

// taskIndex, robot
using SmallH = std::pair<unsigned, std::vector<Assignment>>;

inline auto compareSmallH = [](const SmallH & a, const SmallH & b){
    return (a.second[0]) < (b.second[0]);
};

// heap of assignments that differ by tasks
using BigH = std::list<SmallH>;

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
    BigH bigH;

    static BigH
    buildPartialAssignmentHeap(const std::vector<Assignment> &robots, const TasksVector &tasks,
                               const DistanceMatrix &distanceMatrix);

    static Assignment
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Assignment &robot,
                                const TasksVector &taskVector);

    Assignment extractTop();

    void updateSmallHTop(const std::vector<Assignment> &partialAssignments);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
