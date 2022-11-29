/**
 * @file
 * @brief Contains various functions related to distances between cells of the map.
 * @author Jacopo Zagoli
 * @version 1.0
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#pragma once

#include "Point.h"
#include "ambient/AmbientMapInstance.h"
#include "custom_types.h"

namespace cmapd {

[[nodiscard]] h_table_t load_h_table(const std::filesystem::path & matrixPath, int nCols);

namespace multi_a_star {
/**
 * This function is used by the A* algorithm to compute an estimated distance of a Point from a
 * sequence of goals.
 * @param location The point from which we need to calculate the distance
 * @param label The number of goal locations in goal_sequence that the current A* path has already
 * visited.
 * @param h_table The h-table (table of distances) for the desired map instance.
 * @param goal_sequence The goals the current A* path needs to visit.
 * @return The distance of location to the goals, according to label.
 * @see Lifelong Multi-Agent Path Finding in Large-Scale Warehouses, section 4.1
 */
int compute_h_value(Point location,
                    int label,
                    const h_table_t& h_table,
                    const path_t& goal_sequence);
}  // namespace multi_a_star

}  // namespace cmapd