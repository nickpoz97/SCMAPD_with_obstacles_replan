//
// Created by nicco on 05/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_STATUS_HPP
#define SIMULTANEOUS_CMAPD_STATUS_HPP

#include <unordered_set>

#include "Task.hpp"
#include "AmbientMap.hpp"
#include "AgentInfo.hpp"
#include "PathWrapper.hpp"

class Status{
public:
    Status(AmbientMap ambientMap, const std::vector<AgentInfo> &agents, bool noConflicts, bool online);

    // t is the time when agent does the action
    [[nodiscard]] std::vector<CompressedCoord>
    getValidNeighbors(int agentId, CompressedCoord c, TimeStep t) const;

    [[nodiscard]] const Task & getTask(int i) const;

    std::pair<int, int> update(ExtractedPath pathWrapper);

    [[nodiscard]] bool checkAllConflicts() const;
    [[nodiscard]] bool checkPathWithStatus(const Path &path, int agentId) const;

    [[nodiscard]] const AmbientMap & getAmbient() const;

    [[nodiscard]] bool hasIllegalPositions(const Path &path) const;

    [[nodiscard]] std::vector<int> chooseNRandomTasks(int iterIndex, int n) const;
    [[nodiscard]] std::vector<int> chooseNWorstTasks(int n, Metric mt, const std::vector<int> &coveredTasks) const;

    // todo warning this might not working in online mode
    [[nodiscard]] std::vector<int> chooseTasksFromNWorstAgents(int iterIndex, int n, Metric mt) const;

    [[nodiscard]] const PWsVector & getPathWrappers() const;
    [[nodiscard]] PWsVector & getPathWrappers();

    void setPathWrappers(PWsVector&& other);

    [[nodiscard]] VerbosePath toVerbosePath(int i) const;

    [[nodiscard]] std::string getAgentsSnapshot(int agentId, TimeStep t, CompressedCoord actual) const;
    [[nodiscard]] std::string getTargetSnapshot(CompressedCoord start, CompressedCoord end, CompressedCoord actual) const;

    [[nodiscard]] bool dockingConflict(TimeStep sinceT, CompressedCoord pos, int agentId) const;

    [[nodiscard]] bool allTasksSatisfied() const;

    void updateTasks(std::unordered_map<int, Task> newTasks);

    [[nodiscard]] bool taskIdExists(int taskId) const;

    std::vector<int> getTaskIds() const;

    std::vector<int> getAvailableAgentIds() const;

    [[nodiscard]] std::vector<Path> getPaths(int outAgentId) const;
private:
    const AmbientMap ambient;
    std::unordered_map<int, Task> tasks{};

    PWsVector pathsWrappers;
    bool noConflicts;

    [[nodiscard]] bool
    checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2, TimeStep t1) const;

    static std::vector<PathWrapper> initializePathsWrappers(const std::vector<AgentInfo> &agents);

    [[nodiscard]] bool checkPathConflicts(int i, int j) const;
    static bool checkPathConflicts(const Path &pA, const Path &pB);

    [[nodiscard]] bool isDocking(int agentId, TimeStep t) const;
};

inline std::size_t hash_value(const Status& s){
    const auto& pWs = s.getPathWrappers();
    return boost::hash_range(pWs.cbegin(), pWs.cend());
}

#endif //SIMULTANEOUS_CMAPD_STATUS_HPP