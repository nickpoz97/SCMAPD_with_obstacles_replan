/**
 * @file
 * @brief Contains the pbs method implementation.
 * @author Davide Furlani
 * @author Jacopo Zagoli
 * @version 1.1
 * @date October, 2022
 * @copyright 2022 Jacopo Zagoli, Davide Furlani
 */

#include <vector>

#include "Constraint.h"
#include "Point.h"
#include "a_star/multi_a_star.h"
#include "ambient/AmbientMapInstance.h"
#include "custom_types.h"

#include "Waypoint.hpp"

namespace cmapd::pbs {

    path_t pbs(
            const AmbientMapInstance& instance,
            const std::vector<std::vector<Constraint>> &constraints,
            int aIndex,
            const WaypointsList& waypoints
        ){
    Path waypointsCoords{};
    waypointsCoords.reserve(waypoints.size());
    for(const auto& w : waypoints){
        waypointsCoords.push_back(static_cast<Point>(w));
    }

    return multi_a_star::multi_a_star(aIndex, instance.agents().at(aIndex), waypointsCoords, instance, constraints);
}

}  // namespace cmapd::pbs
