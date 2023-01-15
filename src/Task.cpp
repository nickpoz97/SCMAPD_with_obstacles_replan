#include <fstream>
#include <Task.hpp>

bool operator==(const Task &t1, const Task &t2) {
        return t1.index == t2.index;
}

std::pair<CompressedCoord, CompressedCoord> Task::getCoordinates() const { return {startLoc, goalLoc}; }

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

Task::Task(CompressedCoord startLoc, CompressedCoord goalLoc, const DistanceMatrix& dm) :
    startLoc{startLoc},
    goalLoc{goalLoc},
    releaseTime{},
    index{getNextId()},
    idealGoalTime{releaseTime + dm.getDistance(startLoc, goalLoc)}
{}

int Task::getNextId() {
    static int id = 0;
    auto newId = id;
    ++id;
    return newId;
}

std::vector<Task> loadTasks(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm, char horizontalSep){
    std::ifstream fs (tasksFilePath, std::ios::in);
    std::string line;

    // nTasks line
    std::getline(fs, line);
    size_t nTasks = std::stoi(line);

    std::vector<Task> tasks;
    tasks.reserve(nTasks);

    for (int i = 0 ; i < nTasks ; ++i){
        std::getline(fs, line);

        std::stringstream taskString{line};
        std::string value;

        std::getline(taskString, value, horizontalSep);
        int yBegin = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int xBegin = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int yEnd = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int xEnd = std::stoi(value);

        tasks.emplace_back(dm.from2Dto1D({yBegin, xBegin}), dm.from2Dto1D(yEnd, xEnd), dm);
    }

    return tasks;
}
