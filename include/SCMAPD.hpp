#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include <vector>
#include "Status.hpp"
#include "Assignment.hpp"

class SCMAPD {
public:
    SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
           std::vector<Task> &&tasksVector, Heuristic heuristic, bool debug);

    void solve(TimeStep cutOffTime);

    void printResult() const;

    void printCheckMessage() const;
private:
    Status status;
    Heuristic heuristic;
    BigH bigH;
    bool debug;

    static BigH
    buildPartialAssignmentHeap(const Status &status, Heuristic heuristic);

    static Assignment
    initializePartialAssignment(const Status &status, int taskIndex, const Assignment &robot);

    std::pair<int, Assignment> extractBigHTop();

    void updateSmallHTop(const Assignment &fixedAssignment, int v, std::vector<Assignment> &partialAssignments);

    static void sortBigH(BigH &bigH, Heuristic heuristic);
    static void sortPA(std::vector<Assignment> &pa, int v = 1);

    static inline auto findPA(std::vector<Assignment> &partialAssignments, int agentID) {
        assert(0 <= agentID && agentID < partialAssignments.size());
        auto predicate = [agentID](const Assignment& pa){return pa.getIndex() == agentID;};
        return std::find_if(partialAssignments.begin(), partialAssignments.end(), predicate);
    } ;
};

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                Heuristic heuristic);

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
