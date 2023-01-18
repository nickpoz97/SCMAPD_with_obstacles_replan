//
// Created by nicco on 05/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_STATUS_HPP
#define SIMULTANEOUS_CMAPD_STATUS_HPP

#include <unordered_set>

#include "Task.hpp"
#include "AmbientMap.hpp"

class Status{
public:
    Status(AmbientMap &&ambientMap,
           int nRobots,
           std::vector<Task> && tasks);

    // t is the time when agent does the action
    [[nodiscard]] std::vector<CompressedCoord>
    getValidNeighbors(int agentId, CompressedCoord c, TimeStep t, bool includeHoldAction) const;

    [[nodiscard]] const std::vector<Task> &getTasks() const;
    [[nodiscard]] const std::vector<Path> &getPaths() const;

    [[nodiscard]] const Task & getTask(int i) const;

    void updatePaths(Path &&path, int agentId);

    [[nodiscard]] bool checkAllConflicts() const;
    [[nodiscard]] bool checkPathConflicts(int i, int j) const;
    [[nodiscard]] bool checkPathWithStatus(const Path &path, int agentId) const;

    static bool checkPathConflicts(const Path &pA, const Path &pB) ;
    [[nodiscard]] const DistanceMatrix &getDistanceMatrix() const;

    [[nodiscard]] CompressedCoord holdOrAvailablePos(int agentId, CompressedCoord c, TimeStep t) const;
    [[nodiscard]] TimeStep getLongestPathSize() const;
private:
    const AmbientMap ambient;
    const std::vector<Task> tasksVector;
    std::vector<Path> paths;
    TimeStep longestPathSize = 0;

    [[nodiscard]] bool checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2, TimeStep t1) const;
};


#endif //SIMULTANEOUS_CMAPD_STATUS_HPP
