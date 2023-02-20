#include <algorithm>
#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "BigH.hpp"

SCMAPD::SCMAPD(AmbientMap &&ambientMap, const std::vector<AgentInfo> &agents, std::vector<Task> &&tasksVector,
               Heuristic heuristic, bool debug, PathfindingStrategy strategy) :
    status(std::move(ambientMap), agents, std::move(tasksVector), strategy),
    bigH{agents, status, heuristic},
    debug{debug},
    start{std::chrono::steady_clock::now()}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime) {
    findFirstSolution();
    optimize();

    execution_time = std::chrono::steady_clock::now() - start;
}

void SCMAPD::findFirstSolution() {// extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        auto [k, taskId] = status.update(bigH.extractTop());
        assert(!status.checkAllConflicts());

        bigH.update(k, taskId, status);
    }
}

void SCMAPD::printResult() const{
    auto nAgents = status.getNAgents();

    const auto& pathWrappers = status.getPathWrappers();

    fmt::print("{}\n", nAgents);
    fmt::print("agent\tcost\tttd\tpath\n");
    for(int i = 0 ; i < nAgents ; ++i){
        const auto& path = pathWrappers.getPath(i);

        fmt::print("{}\t{}\t{}\t{}\n", i, pathWrappers.getSpan(i), pathWrappers.getTasksDelay(i) ,status.stringifyPath(path));
    }
    fmt::print("Time:\t{0:.2f}\n", execution_time.count());
    fmt::print("Makespan:\t{}\n", pathWrappers.getMaxSpanCost());
    fmt::print("Total_Travel_Time:\t{}\n", pathWrappers.getTTT());
    fmt::print("Total_Travel_Delay:\t{}\n", pathWrappers.getTTD());
}

void SCMAPD::printCheckMessage() const{
    constexpr std::string_view message{"Conflicts:\t{}"};

    if(status.checkAllConflicts()){
        fmt::print(message, "True");
        return;
    }
    fmt::print(message, "False");
}

void SCMAPD::optimize() {
    const auto backup = status.getPathWrappers();


}

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                Heuristic heuristic, PathfindingStrategy strategy) {
    AmbientMap ambientMap(gridFile, DistanceMatrix{distanceMatrixFile});

    auto robots{loadAgents(agentsFile, ambientMap.getDistanceMatrix())};
    auto tasks{loadTasks(tasksFile, ambientMap.getDistanceMatrix())};

    return {std::move(ambientMap), robots, std::move(tasks), heuristic, false, strategy};
}
