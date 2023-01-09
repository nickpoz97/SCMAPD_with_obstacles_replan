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
    std::vector<CompressedCoord> getValidNeighbors(int agentId, CompressedCoord c, TimeStep t) const;

    const std::vector<Task> &getTasks() const;

    const std::vector<Path> &getPaths() const;

    const Task & getTask(int i) const;

    const std::unordered_set<int>& getUnassignedTasksIndices() const;

    void removeTaskIndex(int i);

    void update(Path &&path, int agentId);

    bool checkAllConflicts(bool printConflicts) const;
    bool checkPathConflicts(int i, int j, bool printConflicts) const;
    bool checkPathConflicts(const Path & pA, const Path& pB, bool printConflicts) const;

    const DistanceMatrix &getDistanceMatrix() const;
private:
    const AmbientMap ambient;
    const std::vector<Task> tasksVector;
    std::vector<Path> paths;
    std::unordered_set<int> unassignedTasksIndices;

    bool checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2, TimeStep t1) const;
};


#endif //SIMULTANEOUS_CMAPD_STATUS_HPP
