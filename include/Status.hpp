//
// Created by nicco on 05/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_STATUS_HPP
#define SIMULTANEOUS_CMAPD_STATUS_HPP

#include <unordered_set>

#include "ambient/AmbientMapInstance.h"
#include "Task.hpp"
#include "Constraint.h"
#include "Assignment.hpp"

using ConstraintsPerAgent = std::vector<std::vector<cmapd::Constraint>>;

class Status{
public:
    Status(cmapd::AmbientMapInstance&& ambientMapInstance,
           std::vector<Assignment> &&robots,
           std::vector<Task> && tasksVector);

    const std::vector<Task> &getTasks() const;

    const std::vector<Assignment> &getAssignments() const;

    const DistanceMatrix& getDistanceMatrix() const;

    const cmapd::AmbientMapInstance & getAmbientMapInstance() const;

    const Task & getTask(int i) const;

    const Assignment& getAssignment(int k) const;
    Assignment& getAssignment(int k);

    const std::unordered_set<int>& getUnassignedTasksIndices() const;

    void removeTaskIndex(int i);

    std::vector<cmapd::Constraint> getOtherConstraints(int k) const;

    void print(FILE *fp = stdout);
private:
    const cmapd::AmbientMapInstance ambientMapInstance;
    const std::vector<Task> tasks;
    std::vector<Assignment> assignments;
    std::unordered_set<int> unassignedTasksIndices;
    ConstraintsPerAgent actualConstraints;
};


#endif //SIMULTANEOUS_CMAPD_STATUS_HPP
