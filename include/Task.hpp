#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include "TypeDefs.hpp"
#include "DistanceMatrix.hpp"

struct Task {
    const Coord startLoc;
    const Coord goalLoc;
    const TimeStep releaseTime;
    const unsigned index;

    friend bool operator==(const Task& t1, const Task& t2);
    [[nodiscard]] TimeStep getIdealGoalTime(const DistanceMatrix& dm) const;

    inline explicit operator std::pair<Coord ,Coord>() const { return {startLoc, goalLoc}; }
};

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
