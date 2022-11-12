//
// Created by nicco on 11/11/2022.
//

#include <numeric>
#include "Robot.hpp"

Robot::Robot(CompressedCoord start, const DistanceMatrix &distanceMatrix) :
    start{start},
    distanceMatrix{distanceMatrix}
    {}

CompressedCoord Robot::getStart() const {
    return start;
}

void Robot::updateTasksAndTTD(SequenceOfReadyTasks &&tasks) {
    readyTasks = std::move(tasks);
    updateTTD();
}

void Robot::updateTTD() {
    auto cumulateDelay = [this](TimeStamp cumulatedDelay, const ReadyTask& readyTask) {
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
