#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <TypeDefs.hpp>
#include <filesystem>
#include <SCMAPD.hpp>
#include "Assignment.hpp"

namespace utils{
    using std::filesystem::path;

    std::vector<Assignment> loadRobots(const path &agentsFilePath, int nCols, char horizontalSep= ',', unsigned int capacity=3);
    std::vector<Task> loadTasks(const path &tasksFilePath, int nCols, char horizontalSep=',');
    SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                    const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                    Heuristic heuristic);
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
