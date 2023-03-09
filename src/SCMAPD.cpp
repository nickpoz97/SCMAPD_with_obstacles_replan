#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "MAPF/PathFinder.hpp"

#include <nlohmann/json.hpp>

SCMAPD::SCMAPD(AmbientMap &&ambientMap, std::vector<AgentInfo> &&agents, std::vector<Task> &&tasksVector,
               Heuristic heuristic) :
    start{std::chrono::steady_clock::now()},
    status(std::move(ambientMap), agents, std::move(tasksVector)),
    bigH{agents, status, heuristic},
    agentInfos{std::move(agents)}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd, Metric mtr) {
    if(!findSolution()){
        throw std::runtime_error("No solution");
    }
    assert(bigH.empty());

    for(int i = 0 ; i < cutOffTime ; ++i){
        bool success = optimize(i, nOptimizationTasks, obj, mtd, mtr);

        // no random elements
        if(!success && mtd == Method::WORST_TASKS){
            break;
        }
    }

    execution_time = std::chrono::steady_clock::now() - start;
}

bool SCMAPD::findSolution() {// extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        auto [k, taskId] = status.update(bigH.extractTop());
        assert(!status.checkAllConflicts());

        if(!bigH.update(k, taskId, status)){
            return false;
        }
    }
    return true;
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

bool SCMAPD::optimize(int iterIndex, int n, Objective obj, Method mtd, Metric mtr) {
    if(n <= 0){
        return false;
    }

    auto PWsBackup{status.getPathWrappers()};

    auto chooseNTasks = [n, mtr, mtd, iterIndex, this](){
        switch (mtd) {
            case Method::WORST_TASKS:
                return status.chooseNWorstTasks(n, mtr);
            case Method::WORST_AGENTS:
                return status.chooseTasksFromNWorstAgents(iterIndex, n, mtr);
            // RANDOM TASKS
            default:
                return status.chooseNRandomTasks(iterIndex, n);
        }
    };

    std::unordered_set<int> chosenTasks{chooseNTasks()};

    // check if it is able to remove tasks
    if(removeTasks(chosenTasks)){
        bigH.addNewTasks(agentInfos, status, std::move(chosenTasks));

        // maintain only better solutions
        if (findSolution() && isBetter(status.getPathWrappers(), PWsBackup, obj)){
            assert(bigH.empty());
            return true;
        }
    }
    status.setPathWrappers(std::move(PWsBackup));
    return false;
}

bool SCMAPD::removeTasks(const std::unordered_set<int> &chosenTasks) {
    for(int agentId = 0 ; agentId < status.getNAgents() ; ++agentId){
        auto& pW = status.getPathWrapper(agentId);
        pW.removeTasksAndWaypoints(chosenTasks);

        auto result {PathFinder::multiAStar(pW.getWaypoints(), pW.getInitialPos(), status, agentId)};

        if(!result.has_value()) {
            return false;
        }

        pW.pathAndWaypointsUpdate(std::move(result.value()));
    }

    return true;
}

bool SCMAPD::isBetter(const PWsVector &newResult, const PWsVector &oldResult, Objective obj) {
    switch (obj) {
        case Objective::MAKESPAN:
            return newResult.getMaxSpanCost() <= oldResult.getMaxSpanCost();
        case Objective::TTD:
            return newResult.getTTD() <= oldResult.getTTD();
        // TTT
        default:
            return newResult.getTTT() <= oldResult.getTTT();
    }
}

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                Heuristic heuristic, PathfindingStrategy strategy) {
    AmbientMap ambientMap(gridFile, distanceMatrixFile);

    return {std::move(ambientMap), loadAgents(agentsFile, ambientMap.getDistanceMatrix()),
            loadTasks(tasksFile, ambientMap.getDistanceMatrix()), heuristic};
}
