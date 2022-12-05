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

using ConstraintsPerAgent = std::vector<std::vector<Constraint>>;

class SCMAPD {
public:
    SCMAPD(
        cmapd::AmbientMapInstance&& ambientMapInstance,
        std::vector<Assignment> &&robots,
        std::vector<Task> && tasksVector
    );

    void solve(Heuristic heuristic, TimeStep cutOffTime);
private:
    cmapd::AmbientMapInstance&& ambientMapInstance;
    std::vector<Assignment> assignments;
    std::vector<Task> tasks;
    std::unordered_set<unsigned int> unassignedTasksIndices;
    BigH bigH;

    ConstraintsPerAgent actualConstraints;

    static BigH
    buildPartialAssignmentHeap(const std::vector<Assignment> &robots, const std::vector<Task> &tasks,
                               const DistanceMatrix &distanceMatrix);

    static Assignment
    initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Assignment &robot,
                                const std::vector<Task> &taskVector);

    Assignment extractTop();

    void updateSmallHTop(int assignmentIndex, int v, std::vector<Assignment> &partialAssignments);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
