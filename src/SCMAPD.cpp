#include <functional>
#include "SCMAPD.hpp"
#include "fmt/color.h"
#include "MAPF/PathFinder.hpp"

#include <nlohmann/json.hpp>

SCMAPD::SCMAPD(AmbientMap ambientMap, const std::vector<AgentInfo>& agents, TaskHandler taskHandler, Heuristic heuristic,
               bool noConflicts, bool online) :
    start{std::chrono::steady_clock::now()},
    taskHandler{std::move(taskHandler)},
    status(std::move(ambientMap), agents, noConflicts, online),
    bigH{heuristic}
    {
        assert(!status.checkAllConflicts());
    }

void SCMAPD::solve(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd, Metric mtr){
    if(status.isOnline()){
        solveOnline(cutOffTime,nOptimizationTasks,obj,mtd,mtr);
        return;
    }
    solveOffline(cutOffTime,nOptimizationTasks,obj,mtd,mtr);
    assert(status.allTasksSatisfied());
}

void SCMAPD::solveOffline(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd, Metric mtr) {
    status.updateTasks(taskHandler.getNextBatch());

    const auto& availableAgents = status.getAvailableAgentIds();
    bigH.addNewTasks(status, status.getAvailableTasksIds(), availableAgents);

    if(!findSolution()){
        throw std::runtime_error("No solution");
    }
    assert(bigH.empty());

    int nIterations = 0;

    auto optimizationBegin = std::chrono::steady_clock::now();
    auto getOptimizationTime = [&optimizationBegin]() {
        return std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - optimizationBegin
        ).count();
    };

    while(getOptimizationTime() < cutOffTime){
        bool success = optimize(nIterations++, nOptimizationTasks, obj, mtd, mtr, availableAgents);

        // no random elements
        if(!success && mtd == Method::WORST_TASKS){
            break;
        }
    }

    execution_time = std::chrono::steady_clock::now() - start;

    const auto& pWs = status.getPathWrappers();
    ttt += pWs.getTTT();
    ttd += pWs.getTTD();
    makespan += pWs.getMaxSpanCost();
    conflicts = status.checkAllConflicts();
    hash = hash_value(status);
}

void SCMAPD::solveOnline(TimeStep cutOffTime, int nOptimizationTasks, Objective obj, Method mtd, Metric mtr){
    while(!taskHandler.noMoreTasks()){
        const auto& availableAgents = status.getAvailableAgentIds();

        status.updateTasks(taskHandler.getNextBatch());

        bigH.addNewTasks(status, status.getAvailableTasksIds(), availableAgents);
        if(!findSolution()){
            throw std::runtime_error("No solution");
        }
        status.incrementTimeStep();
    }
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
    auto nAgents = status.getNAgents();

    const auto& pathWrappers = status.getPathWrappers();

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
                {"path", static_cast<json>(status.toVerbosePath(i)).dump()},
                {"task_ids", json(pW.getSatisfiedTasksIds()).dump()}
            });
        }
    }

    j["stats"] = {
            {"time", fmt::format("{0:.2f}", execution_time.count())},
            {"makespan", makespan},
            {"ttt", ttt},
            {"ttd", ttd},
            {"status_hash", hash},
            {"conflicts", conflicts},
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

bool SCMAPD::optimize(int iterIndex, int n, Objective obj, Method mtd, Metric mtr,
                      const std::vector<int> &availableAgentIds) {
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
        bigH.addNewTasks(status, chosenTasks, availableAgentIds);

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

        Assignment a{pW, status};
        if(!a.removeTasksAndWaypoints(chosenTasks)){
            // some may have been updated
            return false;
        }
        pW = static_cast<PathWrapper>(a);
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
