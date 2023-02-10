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
    SCMAPD(AmbientMap &&ambientMap, const std::vector<AgentInfo> &agents, std::vector<Task> &&tasksVector,
           Heuristic heuristic, bool debug, Strategy strategy);

    void solve(TimeStep cutOffTime);

    void printResult() const;

    void printCheckMessage() const;
private:
    Status status;
    BigH bigH;
    bool debug;

    using time_point = std::chrono::time_point<std::chrono::steady_clock>;
    using duration = std::chrono::duration<double>;

    time_point start;
    duration execution_time{};
};

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                Heuristic heuristic, Strategy strategy);

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
