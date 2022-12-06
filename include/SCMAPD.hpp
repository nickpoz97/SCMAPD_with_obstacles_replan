#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include <vector>
#include "Status.hpp"
#include "Assignment.hpp"

// taskIndex, robot
using SmallH = std::pair<unsigned, std::vector<Assignment>>;

// heap of assignments that differ by tasks
using BigH = std::list<SmallH>;

class SCMAPD {
public:
    SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
           std::vector<Task> &&tasksVector, Heuristic heuristic);

    void solve(Heuristic heuristic, TimeStep cutOffTime);
private:
    Status status;
    Heuristic heuristic;
    BigH bigH;

    static BigH
    buildPartialAssignmentHeap(const Status &status, Heuristic heuristic);

    static Assignment
    initializePartialAssignment(const Status &status, int taskIndex, const Assignment &robot);

    Assignment extractTop();

    void updateSmallHTop(int assignmentIndex, int v, std::vector<Assignment> &partialAssignments);

    static void sortBigH(BigH &bigH, Heuristic heuristic);
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
