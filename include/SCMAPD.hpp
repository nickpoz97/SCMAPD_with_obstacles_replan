#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include <vector>
#include "Status.hpp"
#include "Assignment.hpp"
#include "BigH.hpp"

class SCMAPD {
public:
    SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
           std::vector<Task> &&tasksVector, Heuristic heuristic, bool debug);

    void solve(TimeStep cutOffTime);

    void printResult() const;

    void printCheckMessage() const;
private:
    Status status;
    BigH bigH;
    bool debug;

    static BigH
    buildPartialAssignmentHeap(const Status &status, Heuristic heuristic);
};

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                Heuristic heuristic);

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
