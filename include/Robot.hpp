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
    explicit Robot(CompressedCoord start, const DistanceMatrix &distanceMatrix);

    CompressedCoord getStart() const;

private:
    const CompressedCoord start;
    SequenceOfReadyTasks readyTasks{};
    const DistanceMatrix& distanceMatrix;

    TimeStamp ttd = 0;

    void updateTasksAndTTD(SequenceOfReadyTasks &&tasks);
    void updateTTD();
};


#endif //SIMULTANEOUS_CMAPD_ROBOT_HPP
