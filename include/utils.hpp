#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <TypeDefs.hpp>
#include <filesystem>
#include <SCMAPD.hpp>
#include "Assignment.hpp"

namespace utils{
    DistanceMatrix loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath);
    std::vector<Assignment> loadRobots(const std::filesystem::path &agentsFilePath, int& nCols, char horizontalSep= ',', unsigned int capacity= 1);
    std::vector<Task> loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep=',');
    CompressedCoord from2Dto1D(unsigned int x, unsigned int y, size_t nCols);
}

namespace cmapd{
    std::vector<std::pair<Point, Point>> taskToPointVec(const std::vector<Task>& v, size_t nCols);
    std::vector<Point> robotToPointVec(const std::vector<Assignment>& v, size_t nCols);
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
