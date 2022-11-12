//
// Created by nicco on 12/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include "Robot.hpp"

class SCMAPD {
public:
    SCMAPD(
       DistanceMatrix && distanceMatrix,
       std::vector<Robot> && robots
    );
private:
    const DistanceMatrix distanceMatrix;
    std::vector<Robot> assignment;
};


#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
