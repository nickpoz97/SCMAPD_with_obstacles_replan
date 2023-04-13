#include <fstream>
#include <Task.hpp>

bool operator==(const Task &t1, const Task &t2) {
        return t1.index == t2.index;
}

Task::operator std::string() const {
    auto [firstDiv, lastDiv] = utils::buildDivider("Task");

    return fmt::format(
            "{}\n{}\n{}\n{}\n{}\n{}\n",
            firstDiv,
            fmt::format("Start Coord: {}", startLoc),
            fmt::format("Goal Coord: {}", goalLoc),
            fmt::format("Release Timew: {}", releaseTime),
            fmt::format("Index: {}", index),
            lastDiv
    );
}

Task::Task(CompressedCoord startLoc, CompressedCoord goalLoc, const DistanceMatrix &dm, const TimeStep releaseTime) :
    startLoc{startLoc},
    goalLoc{goalLoc},
    releaseTime{releaseTime},
    index{getNextId()},
    idealGoalTime{releaseTime + dm.getDistance(startLoc, goalLoc)}
{}

int Task::getNextId() {
    static int id = 0;
    auto newId = id;
    ++id;
    return newId;
}

CompressedCoord Task::getStartLoc() const {
    return startLoc;
}

CompressedCoord Task::getGoalLoc() const {
    return goalLoc;
}

TimeStep Task::getReleaseTime() const {
    return releaseTime;
}

int Task::getIndex() const {
    return index;
}

TimeStep Task::getIdealGoalTime() const {
    return idealGoalTime;
}
