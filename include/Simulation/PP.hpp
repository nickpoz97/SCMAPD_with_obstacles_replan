//
// Created by nicco on 05/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_PP_HPP
#define SIMULTANEOUS_CMAPD_PP_HPP

#include "Coord.hpp"
#include "MAPF/SpawnedObstacle.hpp"
#include "AmbientMap.hpp"

namespace PathFinder{
    std::vector<Path> solveWithPP(
        const SpawnedObstaclesSet &sOSet,
        const std::vector<std::vector<CompressedCoord>> &checkpoints,
        const AmbientMap& ambient
    );
}

#endif //SIMULTANEOUS_CMAPD_PP_HPP
