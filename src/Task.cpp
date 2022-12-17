#include <fstream>
#include <Task.hpp>

bool operator==(const Task &t1, const Task &t2) {
        return t1.startLoc == t2.startLoc &&
               t1.goalLoc == t2.goalLoc &&
               t1.releaseTime == t2.releaseTime;
}

TimeStep Task::getIdealGoalTime(const DistanceMatrix &dm) const {
    return releaseTime + dm.getDistance(startLoc, goalLoc);
}

std::pair<Coord, Coord> Task::getCoordinates() const { return {startLoc, goalLoc}; }

Task::operator std::string() const {
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

std::vector<Task> loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep){
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

        int releaseTime = std::getline(taskString, value, ',') ? std::stoi(value) : 0;


        tasks.push_back({{yBegin, xBegin}, {yEnd, xEnd}, releaseTime, i});
    }

    return tasks;
}
