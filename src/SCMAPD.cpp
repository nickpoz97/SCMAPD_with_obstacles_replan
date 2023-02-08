#include <algorithm>
#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "BigH.hpp"

SCMAPD::SCMAPD(AmbientMap&& ambientMap, const std::vector<AgentInfo> &agents,
               std::vector<Task> &&tasksVector, Heuristic heuristic, bool debug) :
    status(std::move(ambientMap), agents, std::move(tasksVector)),
    bigH{agents, status, heuristic},
    debug{debug},
    start{std::chrono::steady_clock::now()}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        auto [k, taskId] = status.update(bigH.extractTop());
        assert(!status.checkAllConflicts());

        bigH.update(k, taskId, status);
    }

    execution_time = std::chrono::steady_clock::now() - start;
}

void SCMAPD::printResult() const{
    fmt::print("{}\n", status.getPaths().size());
    fmt::print("agent\tcost\tttd\tpath\n");
    for(int i = 0 ; i < status.getPaths().size() ; ++i){
        auto& path = status.getPaths()[i];

        fmt::print("{}\t{}\t{}\t{}\n", i, status.getSpanCost(i), status.getTTD(i) ,status.stringifyPath(path));
    }
    fmt::print("Time:\t{0:.2f}\n", execution_time.count());
}

void SCMAPD::printCheckMessage() const{
    constexpr std::string_view message{"Conflicts:\t{}"};

    if(status.checkAllConflicts()){
        fmt::print(message, "True");
        return;
    }
    fmt::print(message, "False");
}

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                       const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                       Heuristic heuristic) {
    AmbientMap ambientMap(gridFile, DistanceMatrix{distanceMatrixFile});

    auto robots{loadAgents(agentsFile, ambientMap.getDistanceMatrix())};
    auto tasks{loadTasks(tasksFile, ambientMap.getDistanceMatrix())};

    return {std::move(ambientMap), robots, std::move(tasks), heuristic, false};
}
