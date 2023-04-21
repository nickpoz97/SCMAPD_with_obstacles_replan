#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include <list>
#include <vector>
#include "Status.hpp"
#include "Assignment.hpp"
#include "BigH.hpp"
#include "TaskHandler.hpp"

class SCMAPD {
public:
    SCMAPD(AmbientMap ambientMap, const std::vector<AgentInfo>& agents, TaskHandler taskHandler, Heuristic heuristic,
           bool noConflicts, bool online);

    void solve(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd, Metric mtr);

    void printResult(bool printAgentsInfo) const;

    void printCheckMessage() const;
private:
    using time_point = std::chrono::time_point<std::chrono::steady_clock>;
    using duration = std::chrono::duration<double>;
    time_point start;
    duration execution_time{};

    TaskHandler taskHandler;

    Status status;
    BigH bigH;

    TimeStep ttd = 0;
    TimeStep ttt = 0;
    TimeStep makespan = 0;
    size_t hash = hash_value(status);
    bool conflicts = false;

    [[nodiscard]] bool findSolution();

    bool optimize(int iterIndex, int n, Objective obj, Method mtd, Metric mtr,
                  const std::vector<int> &availableAgentIds);

    [[nodiscard]] bool removeTasks(const std::vector<int> &chosenTasks);

    static bool isBetter(const PWsVector &newResult, const PWsVector &oldResult, Objective obj);

    void setStats();
};

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
