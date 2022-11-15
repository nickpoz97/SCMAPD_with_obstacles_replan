//
// Created by nicco on 12/11/2022.
//

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

template<Heuristic heuristic>
class SCMAPD {
public:
    SCMAPD(
        DistanceMatrix && distanceMatrix,
        Assignment && robots,
        std::unordered_set<Task> && tasks
    );

    void solve(const PBS &pbs);
private:
    const DistanceMatrix distanceMatrix;
    Assignment assignment;
    std::unordered_set<Task> unassignedTasks;
    PartialAssignmentHeap partialAssignmentsHeap;

    Waypoints insert(const Task &task, const Waypoints &waypoints);
    static PartialAssignmentHeap
    buildPartialAssignmentHeap(const Assignment &robots, const std::unordered_set<Task> &tasks,
                               const DistanceMatrix &distanceMatrix);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
