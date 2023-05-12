#include <functional>
#include "SCMAPD.hpp"
#include "fmt/color.h"
#include "MAPF/PathFinder.hpp"

#include <nlohmann/json.hpp>

SCMAPD::SCMAPD(AmbientMap ambientMap, const std::vector<AgentInfo>& agents, TaskHandler taskHandler, Heuristic heuristic,
               bool noConflicts) :
    start{std::chrono::steady_clock::now()},
    taskHandler{std::move(taskHandler)},
    status(std::move(ambientMap), agents, noConflicts, false),
    bigH{heuristic}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd, Metric mtr) {
    const auto availableAgents = status.getAvailableAgentIds();

    status.updateTasks(taskHandler.getNextBatch());

    if (!bigH.addNewTasks(status, status.getTaskIds()) || !findSolution()) {
        throw std::runtime_error("No solution");;
    }
    int nIterations = 0;

    auto optimizationBegin = std::chrono::steady_clock::now();
    auto getOptimizationTime = [&optimizationBegin]() {
        return std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - optimizationBegin
        ).count();
    };

    while (getOptimizationTime() < cutOffTime) {
        bool success = optimize(nIterations++, nOptimizationTasks, obj, mtd, mtr, availableAgents);

        // no random elements
        if (!success && mtd == Method::WORST_TASKS) {
            break;
        }
    }

    execution_time = std::chrono::steady_clock::now() - start;
    assert(status.allTasksSatisfied());
}

bool SCMAPD::findSolution() {// extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        auto [k, taskId] = status.update(bigH.extractTop());
        assert(!status.checkAllConflicts());

        if(!bigH.update(k, taskId, status)){
            bigH.clear();
            return false;
        }
    }
    return true;
}

void SCMAPD::printResult(bool printAgentsInfo) const{
    const auto& pathWrappers = status.getPathWrappers();

    auto nAgents = pathWrappers.size();

    using namespace nlohmann;
    json j;

    // warning this branch is currently not useful
    if (printAgentsInfo){
        j["agents"] = json::array();

        for(int i = 0 ; i < nAgents ; ++i){
            const auto& pW = pathWrappers[i];

            j["agents"].push_back({
                {"index", i},
                {"ttt", pW.getLastDeliveryTimeStep()},
                {"ttd", pW.getTTD()},
                {"waypoints", getWpsJson(pW.getWaypoints(), status.getDistanceMatrix())},
                {"path", status.toVerbosePath(i)},
                {"task_ids", json(pW.getSatisfiedTasksIds()).dump()}
            });
        }
    }

    j["stats"] = {
            {"time", fmt::format("{0:.2f}", execution_time.count())},
            {"makespan", pathWrappers.getMaxSpanCost()},
            {"ttt", pathWrappers.getTTT()},
            {"ttd", pathWrappers.getTTD()},
            {"status_hash", hash_value(status)},
            {"conflicts", status.checkAllConflicts()},
    };

    fmt::print("{}", j.dump());
}

void SCMAPD::printCheckMessage() const{
    constexpr std::string_view message{"Conflicts:\t{}"};

    if(status.checkAllConflicts()){
        fmt::print(message, "True");
        return;
    }
    fmt::print(message, "False");
}

bool SCMAPD::optimize(int iterIndex, int n, Objective obj, Method mtd, Metric mtr,
                      const std::vector<int> &availableAgentIds) {
    if(n <= 0){
        return false;
    }

    auto PWsBackup{status.getPathWrappers()};

    auto chooseNTasks = [n, mtr, mtd, iterIndex, this](){
        if(mtd == Method::WORST_AGENTS){
            return status.chooseTasksFromNWorstAgents(iterIndex, n, mtr);
        }

        const auto coveredTaskIds = status.getTaskIds();
        if(mtd == Method::WORST_TASKS){
            return status.chooseNWorstTasks(n, mtr, coveredTaskIds);
        }
        // RANDOM_TASKS
        return status.chooseNRandomTasks(iterIndex, n, coveredTaskIds);
    };

    std::vector<int> chosenTasks{chooseNTasks()};

    // check if it is able to remove tasks
    if(removeTasks(chosenTasks)){
        // maintain only better solutions
        if (bigH.addNewTasks(status, chosenTasks) && findSolution() && isBetter(status.getPathWrappers(), PWsBackup, obj)){
            assert(bigH.empty());
            return true;
        }
    }
    status.setPathWrappers(std::move(PWsBackup));
    return false;
}

bool SCMAPD::removeTasks(const std::vector<int> &chosenTasks) {
    PWsVector pWsCopy{status.getPathWrappers()};

    for(auto& pW : pWsCopy){
        Assignment a{pW, status};
        if(!a.removeTasksAndWaypoints({chosenTasks.cbegin(), chosenTasks.cend()})){
            return false;
        }
        pW = static_cast<PathWrapper>(a);
    }

    status.setPathWrappers(std::move(pWsCopy));
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
