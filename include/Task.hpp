//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include <TypeDefs.hpp>

class Task {
public:
    Task(CompressedCoord startLoc, CompressedCoord goalLoc, TimeStamp releaseTime);

    CompressedCoord getStartLoc() const;

    CompressedCoord getGoalLoc() const;

    TimeStamp getReleaseTime() const;

private:
    const CompressedCoord startLoc;
    const CompressedCoord  goalLoc;
    const TimeStamp releaseTime;
};

struct ReadyTask{
    const Task& task;
    PickupDropoffTime pickupDropoffTime;
};


#endif //SIMULTANEOUS_CMAPD_TASK_HPP
