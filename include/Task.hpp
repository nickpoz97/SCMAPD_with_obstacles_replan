#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include "TypeDefs.hpp"
#include "DistanceMatrix.hpp"
#include "utils.hpp"

struct Task {
    const Coord startLoc;
    const Coord goalLoc;
    const TimeStep releaseTime;
    const unsigned index;

    friend bool operator==(const Task& t1, const Task& t2);
    [[nodiscard]] TimeStep getIdealGoalTime(const DistanceMatrix& dm) const;

    inline explicit operator std::pair<Coord ,Coord>() const { return {startLoc, goalLoc}; }

    inline explicit operator std::string() const{
        auto [firstDiv, lastDiv] = utils::buildDivider("Task");

        return fmt::format(
            "{}\n{}\n{}\n{}\n{}\n{}\n",
            firstDiv,
            fmt::format("Start Coord: {}", static_cast<std::string>(startLoc)),
            fmt::format("Goal Coord: {}", static_cast<std::string>(goalLoc)),
            fmt::format("Release Timew: {}", releaseTime),
            fmt::format("Index: {}", index),
            lastDiv
        );
    }
};

std::vector<Task> loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep=',');

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
