//
// Created by nicco on 09/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_AGENTINFO_HPP
#define SIMULTANEOUS_CMAPD_AGENTINFO_HPP

#include "Coord.hpp"

struct AgentInfo{
    const CompressedCoord startPos;
    const int capacity;
    const int index;
};

std::vector<AgentInfo>
loadAgents(const std::filesystem::path &agentsFilePath,
           const DistanceMatrix &dm,
           char horizontalSep=',',
           int capacity=3
);

#endif //SIMULTANEOUS_CMAPD_AGENTINFO_HPP
