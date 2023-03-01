#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "MAPF/PathFinder.hpp"
#include "NoSolution.hpp"
#include "MAPF/NoPathException.hpp"

#include <nlohmann/json.hpp>

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

void SCMAPD::solve(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd) {
    findSolution();
    assert(bigH.empty());

    for(int i = 0 ; i < cutOffTime ; ++i){
        bool success = optimize(i, nOptimizationTasks, obj, mtd);

        // no random elements
        if(!success && mtd == Method::WORST_TASKS){
            break;
        }
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

    using namespace nlohmann;
    json j;

    j["agents"] = json::array();

    for(int i = 0 ; i < nAgents ; ++i){
        const auto& pW = pathWrappers[i];

        j["agents"].push_back({
            {"index", i},
            {"ttt", pW.getLastDeliveryTimeStep()},
            {"ttd", pW.getTTD()},
            {"waypoints", getWpsJson(pW.getWaypoints(), status.getDistanceMatrix())},
            {"path", static_cast<json>(status.toVerbosePath(i)).dump()},
            {"task_ids", json(pW.getSatisfiedTasksIds()).dump()}
        });
    }

    j["stats"] = {
            {"time", fmt::format("{0:.2f}", execution_time.count())},
            {"makespan", pathWrappers.getMaxSpanCost()},
            {"ttt", pathWrappers.getTTT()},
            {"ttd", pathWrappers.getTTD()},
            {"status_hash", hash_value(status)},
            {"conflicts", status.checkAllConflicts()}
    };

    fmt::print("{}", j.dump(1));
}

void SCMAPD::printCheckMessage() const{
    constexpr std::string_view message{"Conflicts:\t{}"};

    if(status.checkAllConflicts()){
        fmt::print(message, "True");
        return;
    }
    fmt::print(message, "False");
}

bool SCMAPD::optimize(int iterIndex, int n, Objective obj, Method mtd) {
    if(n <= 0){
        return false;
    }

    auto PWsBackup{status.getPathWrappers()};

    auto chooseNTasks = [n, obj, mtd, iterIndex, this](){
        switch (mtd) {
            case Method::WORST_TASKS:
                return status.chooseNWorstTasks(n, obj);
            case Method::WORST_AGENTS:
                return status.chooseTasksFromNWorstAgents(iterIndex, n, obj);
            // RANDOM TASKS
            default:
                return status.chooseNRandomTasks(iterIndex, n);
        }
    };

        std::unordered_set<int> chosenTasks{chooseNTasks()};

    try{
        removeTasks(chosenTasks);
        bigH.addNewTasks(agentInfos, status, std::move(chosenTasks));
        findSolution();

        assert(bigH.empty());
        if(!isBetter(status.getPathWrappers(), PWsBackup, obj)){
            throw NoSolution();
        }
    }
    catch(const NoSolution& noSolution){
        status.setPathWrappers(std::move(PWsBackup));
        return false;
    }

    return true;
}

void SCMAPD::removeTasks(const std::unordered_set<int> &chosenTasks) {
    for(int agentId = 0 ; agentId < status.getNAgents() ; ++agentId){
        auto& pW = status.getPathWrapper(agentId);
        pW.removeTasksAndWaypoints(chosenTasks);
        try{
            pW.pathAndWaypointsUpdate(PathFinder::multiAStar(pW.getWaypoints(), pW.getInitialPos(), status, agentId));
        }
        catch(const NoPathException& noPathException){
            // if it is not possible to remove tasks and optimize, stop optimization
            throw NoSolution();
        }
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
    AmbientMap ambientMap(gridFile, distanceMatrixFile);

    return {
        std::move(ambientMap),
        loadAgents(agentsFile, ambientMap.getDistanceMatrix()),
        loadTasks(tasksFile, ambientMap.getDistanceMatrix()),
        heuristic,
        false,
        strategy
    };
}
