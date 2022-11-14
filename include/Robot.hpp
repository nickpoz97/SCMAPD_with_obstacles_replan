//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ROBOT_HPP
#define SIMULTANEOUS_CMAPD_ROBOT_HPP

#include <TypeDefs.hpp>
#include "Task.hpp"

using SequenceOfReadyTasks = std::list<ReadyTask>;

class Robot {
public:
    explicit Robot(CompressedCoord start, unsigned int capacity = 1);
    [[nodiscard]] CompressedCoord getStart() const;

private:
    const CompressedCoord start;
    const unsigned int capacity;

    SequenceOfReadyTasks readyTasks{};
    TimeStamp ttd = 0;

    void updateTasksAndTTD(SequenceOfReadyTasks &&tasks, const DistanceMatrix &distanceMatrix);
    void updateTTD(const DistanceMatrix &distanceMatrix);

public:
    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] const SequenceOfReadyTasks &getReadyTasks() const;

    [[nodiscard]] TimeStamp getTtd() const;
};


#endif //SIMULTANEOUS_CMAPD_ROBOT_HPP
