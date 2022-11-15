#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <TypeDefs.hpp>
#include <filesystem>
#include "PBS.h"

namespace utils{
    DistanceMatrix loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath);
    PBS buildPBS(
        const std::filesystem::path &mapPath,
        const std::filesystem::path &agentsPath,
        int nAgents,
        bool sipp,
        int screen
    );
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
