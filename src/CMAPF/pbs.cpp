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

std::vector<path_t> pbs(const AmbientMapInstance& instance, const std::vector<path_t>& goal_sequences) {
    std::vector<Constraint> constraints{};
    std::vector<path_t> paths{};

    for (int agent = 0; agent < goal_sequences.size(); ++agent) {
        // Computing path
        path_t path = multi_a_star::multi_a_star(
            agent, instance.agents().at(agent), goal_sequences.at(agent), instance, constraints);
        paths.push_back(path);
        // Adding constraints for other agents
        for (int timestep = 0; timestep < path.size(); ++timestep) {
            Point point{path.at(timestep)};
            for (int other_agent = agent + 1; other_agent < instance.num_agents(); ++other_agent) {
                for (moves_t moves{{0, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, 0}};
                     const auto& move : moves) {
                    Point from_where{point + move};
                    if (instance.is_valid(from_where)) {
                        // if this is the last timestep, final should equal to true
                        constraints.emplace_back(Constraint{
                            other_agent, timestep, from_where, point, timestep == path.size() - 1});
                    }
                }
            }
        }
    }
    int makespan{0};
    int cost{0};
    for (const path_t& p : paths) {
        if (static_cast<int>(p.size()) > makespan) makespan = static_cast<int>(p.size());
        cost += static_cast<int>(p.size());
    }

    return paths;
}

std::pair<path_t, std::vector<Constraint>> pbs(
        const AmbientMapInstance& instance,
        const std::vector<Constraint> &constraints,
        int aIndex,
        const WaypointsList& waypoints
        ){
    Path waypointsCoords{};
    waypointsCoords.reserve(waypoints.size());
    for(const auto& w : waypoints){
        waypointsCoords.push_back(w);
    }

    path_t path{multi_a_star::multi_a_star(aIndex, instance.agents().at(aIndex), waypointsCoords, instance, constraints)};

    std::vector<Constraint> newConstraints{};

    // Adding constraints for other agents
    for (int timestep = 0; timestep < path.size(); ++timestep) {
        Point point{path.at(timestep)};
        for (int other_agent = aIndex + 1; other_agent < instance.num_agents(); ++other_agent) {
            for (moves_t moves{{0, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, 0}};
                 const auto& move : moves) {
                Point from_where{point + move};
                if (instance.is_valid(from_where)) {
                    // if this is the last timestep, final should equal to true
                    newConstraints.emplace_back(Constraint{
                            other_agent, timestep, from_where, point, timestep == path.size() - 1});
                }
            }
        }
    }

#ifndef NDEBUG
    for(const auto& c: newConstraints ){
        assert(c.agent != aIndex);
    }
#endif

    return {path, newConstraints};
}

}  // namespace cmapd::pbs
