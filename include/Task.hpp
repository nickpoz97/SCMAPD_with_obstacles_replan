//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include <TypeDefs.hpp>

class Task {
public:
    Task(CompressedCoord startLoc, CompressedCoord goalLoc, TimeStep releaseTime);

    [[nodiscard]] CompressedCoord getStartLoc() const;

    [[nodiscard]] CompressedCoord getGoalLoc() const;

    [[nodiscard]] TimeStep getReleaseTime() const;

private:
    const CompressedCoord startLoc;
    const CompressedCoord  goalLoc;
    const TimeStep releaseTime;
};

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
