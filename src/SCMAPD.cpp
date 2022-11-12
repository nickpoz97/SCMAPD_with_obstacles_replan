//
// Created by nicco on 12/11/2022.
//

#include <filesystem>
#include "SCMAPD.hpp"


SCMAPD::SCMAPD(DistanceMatrix && distanceMatrix,
               std::vector<Robot> && robots) :
    distanceMatrix(std::move(distanceMatrix)),
    assignment(std::move(robots))
    {}
