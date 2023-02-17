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
    Status(AmbientMap &&ambientMap, const std::vector<AgentInfo> &agents, std::vector<Task> &&tasks, Strategy strategy);

    // t is the time when agent does the action
    [[nodiscard]] std::vector<CompressedCoord>
    getValidNeighbors(int agentId, CompressedCoord c, TimeStep t, bool includeHoldAction) const;

    [[nodiscard]] const std::vector<Task> &getTasks() const;
    [[nodiscard]] const Path & getPath(int agentId) const;

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

    [[nodiscard]] TimeStep getSpanCost(int agentId) const;

    [[nodiscard]] bool hasIllegalPositions(const Path &path) const;

    [[nodiscard]] TimeStep getTTD(int agentId) const;

    // this size should be considered an upper bound
    [[nodiscard]] TimeStep getPathsUpperBound() const;

    [[nodiscard]] TimeStep getMaxSpanCost() const;

    [[nodiscard]] TimeStep getTTT() const;

    [[nodiscard]] TimeStep getTTD() const;

    [[nodiscard]] std::optional<int> getMaxPosVisits() const;

    [[nodiscard]] int getNAgents() const;
private:
    const AmbientMap ambient;
    const std::vector<Task> tasksVector;
    std::vector<PathWrapper> pathsWrappers;
    Strategy pathFindingStrategy;

    TimeStep longestPathSize = 0;

    [[nodiscard]] bool checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2, TimeStep t1) const;

    static std::vector<PathWrapper> initializePathsWrappers(const std::vector<AgentInfo> &agents);

};


#endif //SIMULTANEOUS_CMAPD_STATUS_HPP
