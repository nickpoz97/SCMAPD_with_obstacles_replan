#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <TypeDefs.hpp>
#include <filesystem>
#include <SCMAPD.hpp>
#include "PartialAssignment.hpp"

namespace utils{
    DistanceMatrix loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath);
    RobotsVector loadRobots(const std::filesystem::path &agentsFilePath, int& nCols, char horizontalSep= ',', unsigned int capacity= 1);
    TasksVector loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep=',');
    CompressedCoord from2Dto1D(unsigned int x, unsigned int y, size_t nCols);
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
