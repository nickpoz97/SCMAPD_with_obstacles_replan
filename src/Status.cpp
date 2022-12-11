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
    actualConstraints(assignments.size())
    {}

const DistanceMatrix &Status::getDistanceMatrix() const {
    return ambientMapInstance.h_table();
}

const Task & Status::getTask(int i) const {
    return tasks[i];
}

const std::unordered_set<unsigned int> &Status::getUnassignedTasksIndices() const{
    return unassignedTasksIndices;
}

const Assignment &Status::getAssignment(int k) const{
    return assignments[k];
}

std::vector<cmapd::Constraint> Status::getOtherConstraints(int k) const {
    // reserve size
    std::vector<cmapd::Constraint> otherConstraints{};
    size_t reserveSize = 0;
    for(const auto& a : assignments){
        if(a.getIndex() != k) { reserveSize += a.getConstraints().size(); }
    }
    otherConstraints.reserve(reserveSize);
    // copy
    auto outIt = std::back_inserter(otherConstraints);
    for(const auto& a : assignments){
        if (a.getIndex() != k){
            const auto& aCon = a.getConstraints();
            outIt = std::copy(aCon.begin(), aCon.end(), outIt);
        }
    }
    return otherConstraints;
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


