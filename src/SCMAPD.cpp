#include <algorithm>
#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "BigH.hpp"

SCMAPD::SCMAPD(AmbientMap&& ambientMap, const std::vector<AgentInfo> &agents,
               std::vector<Task> &&tasksVector, Heuristic heuristic, bool debug) :
    status(std::move(ambientMap), agents.size(), std::move(tasksVector)),
    bigH{agents, status, heuristic},
    debug{debug}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        auto [taskId, pathWrapper] = bigH.extractTop();
        auto k = pathWrapper.agentId;

        status.updatePaths(std::move(pathWrapper.path), k);
        bigH.update(k, taskId, status);
    }
}

void SCMAPD::printResult() const{
    auto buildPathString = [this](const Path& path){
        static constexpr std::string_view pattern = "({},{})->";

        std::string result{};
        result.reserve(pattern.size() * path.size());

        for(const auto& pos : path){
            auto pos2D = status.getDistanceMatrix().from1Dto2D(pos);
            result.append(fmt::format(pattern, pos2D.row, pos2D.col));
        }

        if(!result.empty()){
            result.resize(result.size() - 2);
        }
        return result;
    };

    fmt::print("agent\tcost\tpath\n");
    for(int i = 0 ; i < status.getPaths().size() ; ++i){
        fmt::print("{}\t{}\t{}\n", i, status.getPaths()[i].size(), buildPathString(status.getPaths()[i]));
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
