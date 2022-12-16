/**
 * @file
 * @brief Contains the pbs solver function.
 * @author Davide Furlani
 * @version 1.0
 * @date November, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */
#pragma once
#include <vector>

#include "ambient/AmbientMapInstance.h"
#include "custom_types.h"
#include "Constraint.h"

namespace cmapd::pbs {

// note: constraints are updated, so it is an in-out parameter
    path_t pbs(
            const AmbientMapInstance& instance,
            const std::vector<Constraint> &constraints,
            int aIndex,
            const WaypointsList& waypoints
);

}  // namespace cmapd::pbs
