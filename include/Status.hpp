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
    Status(AmbientMap &&ambientMap, const std::vector<AgentInfo> &agents, std::vector<Task> &&tasks, PathfindingStrategy strategy);

    // t is the time when agent does the action
    [[nodiscard]] std::vector<CompressedCoord>
    getValidNeighbors(int agentId, CompressedCoord c, TimeStep t, bool includeHoldAction) const;

    [[nodiscard]] const std::vector<Task> &getTasks() const;

    [[nodiscard]] const Task & getTask(int i) const;

    std::pair<int, int> update(ExtractedPath &&pathWrapper);

    [[nodiscard]] bool checkAllConflicts() const;
    [[nodiscard]] bool checkPathConflicts(int i, int j) const;
    [[nodiscard]] bool checkPathWithStatus(const Path &path, int agentId) const;

    static bool checkPathConflicts(const Path &pA, const Path &pB);
    [[nodiscard]] const DistanceMatrix &getDistanceMatrix() const;

    [[nodiscard]] CompressedCoord holdOrAvailablePos(int agentId, CompressedCoord c, TimeStep t) const;
    [[nodiscard]] TimeStep getLongestPathSize() const;

    template<typename T>
    [[nodiscard]] std::string stringifyPath(const T& path) const;

    [[nodiscard]] bool hasIllegalPositions(const Path &path) const;

    [[nodiscard]] std::optional<int> getMaxPosVisits() const;

    [[nodiscard]] int getNAgents() const;

    [[nodiscard]] std::unordered_set<int> chooseNTasks(int n, Objective obj) const;

    std::unordered_set<int> removeTasksFromAgents(const std::unordered_set<int> &rmvTasksIndices);

    [[nodiscard]] const PWsVector & getPathWrappers() const;
    [[nodiscard]] PathWrapper& getPathWrapper(int agentId);
private:
    const AmbientMap ambient;
    const std::vector<Task> tasksVector;
    PWsVector pathsWrappers;
    PathfindingStrategy pathFindingStrategy;

    TimeStep longestPathSize = 0;

    [[nodiscard]] bool checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2, TimeStep t1) const;

    static std::vector<PathWrapper> initializePathsWrappers(const std::vector<AgentInfo> &agents);

};


#endif //SIMULTANEOUS_CMAPD_STATUS_HPP
