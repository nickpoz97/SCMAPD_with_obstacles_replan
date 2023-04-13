#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include "NewTypes.hpp"
#include "DistanceMatrix.hpp"
#include "utils.hpp"

class Task {
public:
    Task(CompressedCoord startLoc, CompressedCoord goalLoc, const DistanceMatrix& dm, const TimeStep releaseTime = 0);

    friend bool operator==(const Task& t1, const Task& t2);

    explicit operator std::string() const;

    [[nodiscard]] CompressedCoord getStartLoc() const;

    [[nodiscard]] CompressedCoord getGoalLoc() const;

    [[nodiscard]] TimeStep getReleaseTime() const;

    [[nodiscard]] int getIndex() const;

    [[nodiscard]] TimeStep getIdealGoalTime() const;

private:
    CompressedCoord startLoc;
    CompressedCoord goalLoc;
    TimeStep releaseTime;
    int index;

    TimeStep idealGoalTime;

    static int getNextId();
};

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
