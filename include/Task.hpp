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

struct TaskHasher{
    std::size_t operator()(const Task& task) const{
        // https://stackoverflow.com/questions/38965931/hash-function-for-3-integers

        auto a = task.getStartLoc();
        auto b = task.getGoalLoc();
        auto c = task.getReleaseTime();

        auto Hab = ((a + b) * (a + b + 1) + b) / 2;

        return ((Hab + c) * (Hab + c + 1) + c) / 2;
    }
};

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
