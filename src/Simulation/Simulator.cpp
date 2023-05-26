//
// Created by nicco on 27/05/2023.
//

#include "Simulation/Simulator.hpp"
#include "PBS.h"

void Simulator::simulate(size_t hash, Strategy strategy) {
    using std::ranges::any_of;

    while(any_of(runningAgents, [](const RunningAgent& ra){return !ra.hasFinished();})){
        // extract obstacles at this timeStep
        auto actualObstacles = obstacles.front();
        obstacles.pop_front();

        // obstacles in that our agent will cross in this iteration
        std::vector<CompressedCoord> foundObstacles{};
        std::ranges::set_intersection(getNextPositions(), actualObstacles, std::back_inserter(foundObstacles));

        if(!foundObstacles.empty()){
            switch (strategy) {
                // warning cannot solve if obstacles is on a target
                case Strategy::RE_PLAN:
                    PBS pbs{generatePBSInstance(actualObstacles), true, 0};
                    pbs.solve(7200);
                    updatePlannedPaths(pbs.getPaths());
                break;
                case Strategy::WAIT:

                break;
            }
        }

        std::ranges::for_each(runningAgents, [](RunningAgent& ra){ra.stepAndUpdate();});
    }
}

std::vector<CompressedCoord> Simulator::getNextPositions() const {

    auto nextPositions =
        runningAgents |
        std::views::transform( [](const auto& ra){return ra.getNextPosition();} ) |
        std::views::filter( [](const auto& optCoord){return optCoord.has_value();} ) |
        std::views::transform( [](const auto& coord){return *coord;} );

    return {nextPositions.begin(), nextPositions.end()};
}

Instance Simulator::generatePBSInstance(const std::vector<CompressedCoord>& obstaclesLocation) const {
    auto checkPointsExtractor = [](const RunningAgent& ra) -> Path {
        std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

        std::ranges::copy(
            ra.getPlannedCheckpoints(),
            std::back_inserter(checkPoints)
        );

        return checkPoints;
    };

    std::vector<Path> agentsCheckpoints{};

    std::ranges::transform(runningAgents, std::back_inserter(agentsCheckpoints), checkPointsExtractor);
    assert(agentsCheckpoints.size() == runningAgents.size());

    // extract grid
    auto grid{ambientMap.getGrid()};
    assert(grid.size() == ambientMap.getNRows() * ambientMap.getNCols());

    // each position with an obstacle is considered an obstacle
    std::ranges::for_each(obstaclesLocation, [&grid](CompressedCoord obstacle){grid[obstacle] = true;});

    return {grid, agentsCheckpoints, ambientMap.getNRows(), ambientMap.getNCols()};
}

void Simulator::updatePlannedPaths(const std::vector<Path>& paths) {
    for (int i = 0 ; i < paths.size() ; i++){
        assert(i = runningAgents[i].getAgentId());
        runningAgents[i].setPlannedPath(paths[i]);
    }
}
