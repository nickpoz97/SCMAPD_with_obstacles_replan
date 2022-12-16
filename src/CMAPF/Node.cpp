/**
 * @file
 * @brief Contains the cbs Node implementation.
 * @author Jacopo Zagoli
 * @version 1.0
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#include "Node.h"

#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include "Conflict.h"
#include "ConflictType.h"
#include "Constraint.h"
#include "Point.h"
#include "a_star/multi_a_star.h"
#include "ambient/AmbientMapInstance.h"
#include "custom_types.h"

namespace cmapd::cbs {

/**
 * Get a Point in a path at a given timestamp.
 * @param path The path to be analyzed.
 * @param timestep The timestep at which we request a Point.
 * @return the Point at the given timestep. If the timestep is longer than the path, returns the
 * last point.
 */
Point get_position(const path_t& path, int timestep) {
    if (timestep < 0) {
        throw std::invalid_argument{"timestep must be greater than zero."};
    }
    if (timestep < std::ssize(path)) {
        return path.at(timestep);
    } else {
        return path.back();
    }
}

Node::Node(const AmbientMapInstance& instance,
           std::vector<path_t> goal_sequences,
           std::vector<std::vector<Constraint>> &&constraints)
    : m_constraints{std::move(constraints)} {
    for (int i = 0; i < std::ssize(goal_sequences); ++i) {
        auto start_location = goal_sequences.at(i).at(0);
        // remove start location from goal_sequence
        goal_sequences.at(i).erase(goal_sequences.at(i).cbegin());
        m_paths.push_back(cmapd::multi_a_star::multi_a_star(
            i, start_location, goal_sequences.at(i), instance, m_constraints));
    }
}

std::vector<int> Node::lengths() const {
    std::vector<int> lengths;
    for (const auto& path : m_paths) {
        lengths.emplace_back(std::ssize(path));
    }
    return lengths;
}

int Node::makespan() const {
    int makespan = std::numeric_limits<int>::min();
    for (const auto& path : m_paths) {
        int size = static_cast<int>(std::ssize(path));
        makespan = size > makespan ? size : makespan;
    }
    return makespan;
}

std::optional<Conflict> Node::first_conflict() const {
    auto n_paths = std::ssize(m_paths);
    for (int i = 0; i < n_paths; ++i) {
        for (int j = i + 1; j < n_paths; ++j) {
            auto opt_conflict = detect_conflict(i, j, m_paths.at(i), m_paths.at(j));
            if (opt_conflict) {
                return opt_conflict;
            }
        }
    }
    return {};
}

std::optional<Conflict> Node::detect_conflict(int first_agent,
                                              int second_agent,
                                              const path_t& first_path,
                                              const path_t& second_path) {
    auto length = std::ssize(first_path) > std::ssize(second_path) ? std::ssize(first_path)
                                                                   : std::ssize(second_path);
    for (int timestep = 0; timestep < length; ++timestep) {
        // Check for vertex collision
        auto first_pos = get_position(first_path, timestep);
        auto second_pos = get_position(second_path, timestep);
        if (first_pos == second_pos) {
            return Conflict{
                first_agent, second_agent, timestep, first_pos, second_pos, ConflictType::VERTEX};
        }
        // Check for edge collision except for when we are in the last timestep
        if (timestep < length - 1) {
            auto first_next_pos = get_position(first_path, timestep + 1);
            auto second_next_pos = get_position(second_path, timestep + 1);
            if (first_pos == second_next_pos && second_pos == first_next_pos) {
                return Conflict{first_agent,
                                second_agent,
                                timestep + 1,
                                first_pos,
                                first_next_pos,
                                ConflictType::EDGE};
            }
        }
    }
    return {};
}

std::vector<path_t> Node::get_paths() const { return m_paths; }

int Node::cost() const {
    int cost = 0;
    for (const auto& path : m_paths) {
        cost += static_cast<int>(std::ssize(path));
    }
    return cost;
}

int Node::num_conflicts() const {
    auto n_paths = std::ssize(m_paths);
    int num_conflicts = 0;
    for (int i = 0; i < n_paths; ++i) {
        for (int j = i + 1; j < n_paths; ++j) {
            if (detect_conflict(i, j, m_paths.at(i), m_paths.at(j))) {
                ++num_conflicts;
            }
        }
    }
    return num_conflicts;
}

}  // namespace cmapd::cbs