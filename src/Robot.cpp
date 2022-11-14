//
// Created by nicco on 11/11/2022.
//

#include <numeric>
#include "Robot.hpp"

Robot::Robot(CompressedCoord start, unsigned int capacity) :
    start{start},
    capacity{capacity}
    {}

CompressedCoord Robot::getStart() const {
    return start;
}

void Robot::updateTasksAndTTD(SequenceOfReadyTasks &&tasks, const DistanceMatrix &distanceMatrix) {
    readyTasks = std::move(tasks);
    updateTTD(distanceMatrix);
}

void Robot::updateTTD(const DistanceMatrix &distanceMatrix) {
    auto cumulateDelay = [&distanceMatrix](TimeStamp cumulatedDelay, const ReadyTask& readyTask) {
        const auto& task = readyTask.task;

        auto delay =
                readyTask.pickupDropoffTime.second -
                task.getReleaseTime() -
                distanceMatrix[task.getStartLoc()][task.getGoalLoc()];

        return cumulatedDelay + delay;
    };

    ttd = std::accumulate(
        readyTasks.cbegin(),
        readyTasks.cend(),
        0,
        cumulateDelay
    );
}

unsigned int Robot::getCapacity() const {
    return capacity;
}

const SequenceOfReadyTasks &Robot::getReadyTasks() const {
    return readyTasks;
}

TimeStamp Robot::getTtd() const {
    return ttd;
}
