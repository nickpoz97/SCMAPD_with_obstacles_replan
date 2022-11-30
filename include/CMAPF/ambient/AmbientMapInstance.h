/**
 * @file
 * @brief Contains the class AmbientMapInstance.
 * @author Davide Furlani
 * @version 1.0
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#pragma once
#include <filesystem>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "Point.h"
#include "ambient/AmbientMap.h"
#include "DistanceMatrix.hpp"

namespace cmapd {
/**
 * @class AmbientMapInstance
 * @brief This class extends AmbientMap with the initial position of agents and tasks, and
 * with a distance matrix.
 */
class AmbientMapInstance final : public AmbientMap {
  private:
    std::vector<Point> m_agents;
    std::vector<std::pair<Point, Point>> m_tasks;
    DistanceMatrix m_h_table;

  public:
    // given map, agents, tasks and h_table it build a runtime instance
    explicit AmbientMapInstance(
        const AmbientMap& map,
        std::vector<Point> a,
        std::vector<std::pair<Point, Point>> t,
        DistanceMatrix&& distanceMatrix
    );

    /**
     * Method that return the number of agents in the map
     * @returns the number of agents in the map
     */
    [[nodiscard]] int num_agents() const;
    /**
     * Method that return the number of tasks in the map
     * @returns the number of tasks in the map
     */
    [[nodiscard]] int num_tasks() const;
    /**
     * Checks if a position can be a valid move position in the map
     * @param[in] p The point to check
     * @returns True if the given Point is inside the map and it's not a wall, false otherwise
     */
    [[nodiscard]] bool is_valid(Point p) const override;
    /**
     * Method that return a string representing the structure of the map
     * @returns a string representing the structure of the map
     */
    [[nodiscard]] std::string to_string() const override;
    /**
     * Method that return the list of tasks of the instance
     * @returns the list of tasks of the instance
     */
    [[nodiscard]] std::vector<std::pair<Point, Point>> tasks() const;
    /**
     * Method that returns the list of agents of the instance.
     * @return the list of agents of the instance.
     */
    [[nodiscard]] const std::vector<Point>& agents() const;
    /**
     * Method that returns the m_h_table of the instance.
     * @return the m_h_table
     */
    [[nodiscard]] const h_table_t& h_table() const;
    /**
     * Stream operator
     * @param os output stream
     * @param instance instance to send to stream
     * @return stream
     */
    friend std::ostream& operator<<(std::ostream& os, const AmbientMapInstance& instance);
    /**
     * Method that modify the map to add a wall in a given point in the map
     * @param p the point to tranform to a wall
     */
    void wall(Point p);
};
}  // namespace cmapd

