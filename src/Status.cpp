//
// Created by nicco on 05/12/2022.
//

#include <boost/iterator/counting_iterator.hpp>
#include <fmt/core.h>
#include "Status.hpp"

Status::Status(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
               std::vector<Task> &&tasksVector) :
    ambientMapInstance(std::move(ambientMapInstance)),
    tasks(std::move(tasksVector)),
    assignments(std::move(robots)),
    unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasks.size())),
    constraints(assignments.size())
    {}

const DistanceMatrix &Status::getDistanceMatrix() const {
    return ambientMapInstance.h_table();
}

const Task & Status::getTask(int i) const {
    return tasks[i];
}

const std::unordered_set<int> &Status::getUnassignedTasksIndices() const{
    return unassignedTasksIndices;
}

const Assignment &Status::getAssignment(int k) const{
    return assignments[k];
}

std::vector<std::vector<cmapd::Constraint>> Status::getOtherConstraints(int k){
    return constraints;
}

const std::vector<Task> &Status::getTasks() const {
    return tasks;
}

const std::vector<Assignment> &Status::getAssignments() const {
    return assignments;
}

const cmapd::AmbientMapInstance & Status::getAmbientMapInstance() const {
    return ambientMapInstance;
}

Assignment &Status::getAssignment(int k) {
    return assignments[k];
}

void Status::removeTaskIndex(int i) {
    unassignedTasksIndices.erase(i);
}

void Status::print(FILE *fp) {
    static int nCalls = 0;

    if(nCalls == 0) {
        for(const auto& t : tasks){
            fmt::print("{}", static_cast<std::string>(t));
        }
    }

    fmt::print(fp, "iteration {}\n", nCalls++);
    for (const auto& a : assignments){
        fmt::print("{}", static_cast<std::string>(a));
    }
}

int Status::update(Assignment&& a) {
    auto k = a.getIndex();
    constraints[k] = a.getConstraints(ambientMapInstance);
    assignments[k] = std::move(a);
    return k;
}

bool Status::checkCollisions() const{
    for(int i = 0 ; i < assignments.size(); ++i){
        for(int j = i+1 ; j < assignments.size(); ++j){
            if(Assignment::hasConflicts(assignments[i], assignments[j])){
                return true;
            }
        }
    };
    return false;
}


