//
// Created by nicco on 12/04/2023.
//

#ifndef SIMULTANEOUS_CMAPD_TASKHANDLER_HPP
#define SIMULTANEOUS_CMAPD_TASKHANDLER_HPP


#include <filesystem>
#include <unordered_map>
#include "NewTypes.hpp"
#include "DistanceMatrix.hpp"
#include "Task.hpp"

class TaskHandler {
public:
    TaskHandler(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm);
    TaskHandler(
        const std::filesystem::path &tasksFilePath,
        const DistanceMatrix &dm,
        bool isNumerator,
        int frequency
    );

    std::unordered_map<int, Task> getNextBatch();
    [[nodiscard]] bool noMoreTasks() const;

private:
    const std::vector<Task> allTasks;
    TimeStep t = 0;
    int firstTasksBatchIndex = 0;

    static std::vector<Task>
    loadTasks(const std::filesystem::path &tasksFilePath, const DistanceMatrix &dm, bool isMoreThanOne,
              int frequency);
};


#endif //SIMULTANEOUS_CMAPD_TASKHANDLER_HPP
