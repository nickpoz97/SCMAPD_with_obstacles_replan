/**
 * @file
 * @brief Contains the implementation of class AmbientMapInstance.
 * @author Davide Furlani
 * @version 1.0
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#include "ambient/AmbientMapInstance.h"

#include <filesystem>
#include <utility>
#include <vector>

#include "Point.h"

namespace cmapd {

AmbientMapInstance::AmbientMapInstance(
    const AmbientMap &map,
    std::vector<Point> a,
    std::vector<std::pair<Point, Point>> t,
    DistanceMatrix&& distanceMatrix
) : AmbientMap(map), m_agents(std::move(a)), m_tasks(std::move(t)), m_h_table(std::move(distanceMatrix)){

    for (auto agent : m_agents) {
        m_grid[agent.row][agent.col] = 'a';
    }

    for (auto task : m_tasks) {
        m_grid[task.first.row][task.first.col] = 't';
        m_grid[task.second.row][task.second.col] = 't';
    }
}

int AmbientMapInstance::num_agents() const {
    if (m_agents.size() > std::numeric_limits<int>::max())
        throw std::overflow_error("The number of m_agents is larger than a int.");
    return static_cast<int>(m_agents.size());
}

int AmbientMapInstance::num_tasks() const {
    if (m_tasks.size() > std::numeric_limits<int>::max())
        throw std::overflow_error("The number of m_tasks is larger than a int.");
    return static_cast<int>(m_tasks.size());
}

bool AmbientMapInstance::is_valid(Point p) const {
    return p.row >= 0 && p.row < this->rows_number() && p.col >= 0 && p.col < this->columns_number()
           && m_grid[p.row][p.col] != '#';
}

std::string AmbientMapInstance::to_string() const {
    std::string s;
    for (int r = 0; r < this->rows_number(); r++) {
        for (int c = 0; c < this->columns_number(); c++) {
            if (m_grid[r][c] == 'O')
                s += ' ';
            else
                s += m_grid[r][c];
        }
        s += "\n";
    }
    return s;
}

std::vector<std::pair<Point, Point>> AmbientMapInstance::tasks() const { return m_tasks; }
const std::vector<Point>& AmbientMapInstance::agents() const { return m_agents; }
const DistanceMatrix& AmbientMapInstance::h_table() const { return m_h_table; }
std::ostream& operator<<(std::ostream& os, const AmbientMapInstance& instance) {
    os << instance.to_string();
    return os;
}

void AmbientMapInstance::wall(Point p) {
    if (is_valid(p)) m_grid.at(p.row).at(p.col) = '#';
}

}  // namespace cmapd
