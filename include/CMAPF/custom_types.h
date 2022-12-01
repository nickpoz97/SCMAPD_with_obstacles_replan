/**
 * @file
 * @brief Contains type aliases used by the project.
 * @author Jacopo Zagoli
 * @version 1.0
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#pragma once
#include <utility>
#include <vector>

#include "Point.h"

namespace cmapd {
/// A type alias for a path, provided for ease of use.
using path_t = std::vector<Point>;
/// A type alias for a vector of "moves", or offset to a Point
using moves_t = std::vector<std::pair<int, int>>;
}  // namespace cmapd
