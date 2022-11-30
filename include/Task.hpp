//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include "TypeDefs.hpp"
#include "DistanceMatrix.hpp"

struct Task {
    const CompressedCoord startLoc;
    const CompressedCoord goalLoc;
    const TimeStep releaseTime;
    const unsigned index;

    friend bool operator==(const Task& t1, const Task& t2);
    [[nodiscard]] TimeStep getIdealGoalTime(const DistanceMatrix& dm) const;
};

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
