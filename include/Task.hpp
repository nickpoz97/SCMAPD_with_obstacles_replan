//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include <unordered_set>
#include <TypeDefs.hpp>

struct Task {
    const CompressedCoord startLoc;
    const CompressedCoord goalLoc;
    const TimeStep releaseTime;

    friend bool operator==(const Task& t1, const Task& t2);
};

using TasksVector = std::vector<Task>;

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
