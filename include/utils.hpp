//
// Created by nicco on 12/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <TypeDefs.hpp>
#include <filesystem>

namespace utils{
    static DistanceMatrix loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath);
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
