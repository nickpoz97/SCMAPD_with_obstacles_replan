#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include "TypeDefs.hpp"
#include "DistanceMatrix.hpp"
#include "utils.hpp"

struct Task {
    Task(CompressedCoord startLoc, CompressedCoord goalLoc, const DistanceMatrix& dm);

    const CompressedCoord startLoc;
    const CompressedCoord goalLoc;
    const TimeStep releaseTime;
    const int index;

    const TimeStep idealGoalTime;

    friend bool operator==(const Task& t1, const Task& t2);

    [[nodiscard]] std::pair<CompressedCoord, CompressedCoord> getCoordinates() const;

    explicit operator std::string() const;
    explicit operator std::pair<CompressedCoord, CompressedCoord>() const{return getCoordinates();}

private:
    static int getNextId();
};

std::vector<Task> loadTasks(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm, char horizontalSep=',');

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
