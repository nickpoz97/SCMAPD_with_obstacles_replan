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

std::vector<Task>
loadTasks(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm) {
    std::ifstream fs (tasksFilePath, std::ios::in);

    if(!fs.is_open()){
        throw std::runtime_error("Tasks file does not exist");
    }

    std::string line;

    // nTasks line
    std::getline(fs, line);
    size_t nTasks = std::stoi(line);

    std::vector<Task> tasks;
    tasks.reserve(nTasks);

    for (int i = 0 ; i < nTasks ; ++i){
        std::getline(fs, line);
        static constexpr TimeStep releaseTime = 0;

        std::stringstream taskString{line};
        std::string value;

        static constexpr char horizontalSep = ',';

        std::getline(taskString, value, horizontalSep);
        int yBegin = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int xBegin = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int yEnd = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int xEnd = std::stoi(value);

        tasks.emplace_back(dm.from2Dto1D({yBegin, xBegin}), dm.from2Dto1D(yEnd, xEnd), dm, releaseTime);
    }

    return tasks;
}
