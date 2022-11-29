/**
 * @file
 * @brief Contains the struct Constraint.
 * @author Jacopo Zagoli
 * @version 1.0
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#pragma once
#include "Point.h"

namespace cmapd {
/// @struct Constraint
/// @brief This struct models a constraint that prohibits an agent from moving in a certain way.
/// Formally this is an edge-constraint, but we also use it to model a vertex constraint.
struct Constraint {
    /// The constrained agent.
    int agent;
    /// The moment in time in which the agent is prohibited from doing a move.
    int timestep;
    /// The cell from which the agent can't move.
    Point from_position;
    /// The cell to which the agent can't arrive.
    Point to_position;
    /// If it's true, indicates that no other agent can move in this way later than timestep. This
    /// field is used only by the PBS algorithm.
    bool final{false};
    /// Equality operator for algorithms.
    [[nodiscard]] bool operator==(const Constraint& rhs) const = default;
};

}  // namespace cmapd