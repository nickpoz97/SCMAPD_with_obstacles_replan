//
// Created by nicco on 12/04/2023.
//

#include <fstream>
#include "TaskHandler.hpp"

TaskHandler::TaskHandler(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm) :
    allTasks{loadTasks(tasksFilePath, dm, false, 0)}
{}

TaskHandler::TaskHandler(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm, bool isNumerator,
                         int frequency) :
    allTasks(loadTasks(tasksFilePath, dm, isNumerator, frequency))
{}

std::vector<Task>
TaskHandler::loadTasks(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm, bool isMoreThanOne,
                       int frequency) {
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
        TimeStep releaseTime = (frequency == 0) ? 0 : (isMoreThanOne ? i / frequency : i * frequency);

        std::stringstream taskString{line};
        std::string value;

        constexpr char horizontalSep = ',';

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

std::unordered_map<int, Task> TaskHandler::getNextBatch() {
    std::unordered_map<int, Task> batch;

    int i;
    for(i = firstTasksBatchIndex ; i < std::ssize(allTasks) && allTasks.at(i).getReleaseTime() == t ; ++i){
        auto& task = allTasks.at(i);
        batch.emplace(task.getIndex(), task);
    }

    ++t;
    firstTasksBatchIndex = i;

    return batch;
}

bool TaskHandler::noMoreTasks() const {
    return firstTasksBatchIndex >= allTasks.size();
}
