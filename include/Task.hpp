#ifndef SIMULTANEOUS_CMAPD_TASK_HPP
#define SIMULTANEOUS_CMAPD_TASK_HPP

#include "TypeDefs.hpp"
#include "DistanceMatrix.hpp"
#include "utils.hpp"

struct Task {
    const Coord startLoc;
    const Coord goalLoc;
    const TimeStep releaseTime;
    const int index;

    friend bool operator==(const Task& t1, const Task& t2);
    [[nodiscard]] TimeStep getIdealGoalTime(const DistanceMatrix& dm) const;

    [[nodiscard]] std::pair<Coord, Coord> getCoordinates() const;

    explicit operator std::string() const;
    explicit operator std::pair<Coord, Coord>() const{return getCoordinates();}
};

std::vector<Task> loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep=',');

#endif //SIMULTANEOUS_CMAPD_TASK_HPP
