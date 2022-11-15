//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include <unordered_set>
#include <TypeDefs.hpp>

struct Task {
    const CompressedCoord startLoc;
    const CompressedCoord  goalLoc;
    const TimeStep releaseTime;

    friend bool operator==(const Task& t1, const Task& t2);
};

struct TaskHasher{
    std::size_t operator()(const Task& task) const;
};

using TaskSet = std::unordered_set<Task, TaskHasher>;

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
