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
    debug{debug}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        assert(!status.checkAllConflicts());
        auto [taskId, pathWrapper] = bigH.extractTop();
        auto k = pathWrapper.agentId;

        status.updatePaths(std::move(pathWrapper.path), k);
        bigH.update(k, taskId, status);
    }
}

void SCMAPD::printResult() const{
    fmt::print("agent\tcost\tpath\n");
    for(int i = 0 ; i < status.getPaths().size() ; ++i){
        auto& p = status.getPaths()[i];

        fmt::print("{}\t{}\t{}\n", i, p.size(), status.stringifyPath(p));
    }
}

void SCMAPD::printCheckMessage() const{
    if(!status.checkAllConflicts()){
        fmt::print(fmt::emphasis::bold | fg(fmt::color::green), "No collisions\n");
    }
}

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                       const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                       Heuristic heuristic) {
    DistanceMatrix dm{distanceMatrixFile};
    AmbientMap ambientMap(gridFile, std::move(dm));

    auto robots{loadAgents(agentsFile, ambientMap.getDistanceMatrix())};
    auto tasks{loadTasks(tasksFile, ambientMap.getDistanceMatrix())};

    return {std::move(ambientMap), robots, std::move(tasks), heuristic, false};
}
