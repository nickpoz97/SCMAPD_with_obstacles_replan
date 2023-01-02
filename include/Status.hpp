//
// Created by nicco on 05/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_STATUS_HPP
#define SIMULTANEOUS_CMAPD_STATUS_HPP

#include <unordered_set>

#include "Task.hpp"
#include "Assignment.hpp"
#include "AmbientMap.hpp"

using ConstraintsPerAgent = std::vector<std::vector<cmapd::Constraint>>;

class Status{
public:
    Status(AmbientMap &&ambientMap,
           std::vector<Assignment> &&robots,
           std::vector<Task> && tasksVector);

    const std::vector<Task> &getTasks() const;

    const std::vector<Assignment> &getAssignments() const;

    const DistanceMatrix& getDistanceMatrix() const;

    const cmapd::AmbientMapInstance & getAmbientMapInstance() const;

    const Task & getTask(int i) const;

    const Assignment& getAssignment(int k) const;

    const std::unordered_set<int>& getUnassignedTasksIndices() const;

    void removeTaskIndex(int i);

    const std::vector<std::vector<cmapd::Constraint>> & getConstraints() const;

    void print(FILE *fp = stdout);

    int update(Assignment&& a);

    bool printCollisions() const;

private:
    const cmapd::AmbientMapInstance ambientMapInstance;
    const std::vector<Task> tasks;
    std::vector<Assignment> assignments;
    std::unordered_set<int> unassignedTasksIndices;
    std::vector<std::vector<cmapd::Constraint>> constraints;
};


#endif //SIMULTANEOUS_CMAPD_STATUS_HPP
