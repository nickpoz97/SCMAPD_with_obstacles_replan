//
// Created by nicco on 05/06/2023.
//

#include "Simulation/PP.hpp"
#include "MAPF/PathFinder.hpp"

std::vector<Path> PathFinder::solveWithPP(
        const SpawnedObstaclesSet &sOSet,
        const std::vector<std::vector<CompressedCoord>> &checkpoints,
        const AmbientMap& ambient
    ){

    std::vector<Path> paths;
    paths.reserve(checkpoints.size());

    for(const auto& agentCheckpoints : checkpoints){
        int i = 0;
        // <order, location>
        std::vector<std::pair<int, CompressedCoord>> goals;
        goals.reserve(checkpoints.size());
        std::ranges::transform(
            agentCheckpoints,
            std::back_inserter(goals),
            [&i](CompressedCoord cc) -> std::pair<int, CompressedCoord> {return {i++, cc};}
        );

        auto result = PathFinder::multiAStar(
            goals, agentCheckpoints.front(), paths, ambient, sOSet
        );

        if(!result){
            throw std::runtime_error("No path");
        }
        paths.push_back(*result);
    }

    return paths;
}

