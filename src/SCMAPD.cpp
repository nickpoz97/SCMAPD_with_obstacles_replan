#include <algorithm>
#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "BigH.hpp"
#include "MAPF/PathFinder.hpp"

SCMAPD::SCMAPD(AmbientMap &&ambientMap, std::vector<AgentInfo> &&agents, std::vector<Task> &&tasksVector,
               Heuristic heuristic, bool debug, PathfindingStrategy strategy) :
    start{std::chrono::steady_clock::now()},
    status(std::move(ambientMap), agents, std::move(tasksVector), strategy),
    bigH{agents, status, heuristic},
    debug{debug},
    agentInfos{std::move(agents)}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime, int nOptimizationTasks, Objective obj) {
    findSolution();
    assert(bigH.empty());

    bool success = true;
    for(int i = 0 ; i < cutOffTime && success ; ++i){
        success = optimize(nOptimizationTasks, obj);
    }

    execution_time = std::chrono::steady_clock::now() - start;
}

void SCMAPD::findSolution() {// extractBigHTop takes care of tasks indices removal
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

bool SCMAPD::optimize(int n, Objective obj) {
    if(n <= 0){
        return false;
    }

    auto PWsBackup{status.getPathWrappers()};

    auto chosenTasks = status.chooseNTasks(n, obj);
    removeTasks(chosenTasks);

    bigH.addNewTasks(agentInfos, status, std::move(chosenTasks));

    findSolution();
    assert(bigH.empty());

    if(!isBetter(status.getPathWrappers(), PWsBackup, obj)){
        status.setPathWrappers(std::move(PWsBackup));
        return false;
    }
    return true;
}

void SCMAPD::removeTasks(const std::unordered_set<int> &chosenTasks) {
    for(int agentId = 0 ; agentId < status.getNAgents() ; ++agentId){
        auto& pW = status.getPathWrapper(agentId);
        pW.removeTasksAndWPs(chosenTasks);
        pW.PathAndWPsUpdate(PathFinder::multiAStar(pW.getWaypoints(), pW.getInitialPos(), status, agentId));
    }
}

bool SCMAPD::isBetter(const PWsVector &newResult, const PWsVector &oldResult, Objective obj) {
    switch (obj) {
        case Objective::MAKESPAN:
            return newResult.getMaxSpanCost() <= oldResult.getMaxSpanCost();
        case Objective::TTD:
            return newResult.getTTD() <= oldResult.getTTD();
    }
}

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                Heuristic heuristic, PathfindingStrategy strategy) {
    AmbientMap ambientMap(gridFile, DistanceMatrix{distanceMatrixFile});

    auto robots{loadAgents(agentsFile, ambientMap.getDistanceMatrix())};
    auto tasks{loadTasks(tasksFile, ambientMap.getDistanceMatrix())};

    return {std::move(ambientMap), std::move(robots), std::move(tasks), heuristic, false, strategy};
}
