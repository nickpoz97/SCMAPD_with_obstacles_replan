#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <TypeDefs.hpp>
#include <filesystem>
#include "PBS.h"

namespace utils{
    DistanceMatrix loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath);
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
