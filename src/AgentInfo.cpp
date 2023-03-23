//
// Created by nicco on 09/01/2023.
//

#include <filesystem>
#include <fstream>
#include "DistanceMatrix.hpp"
#include "AgentInfo.hpp"

std::vector<AgentInfo>
loadAgents(const std::filesystem::path &agentsFilePath, const DistanceMatrix &dm, char horizontalSep,
           int capacity) {
    std::ifstream fs (agentsFilePath, std::ios::in);

    if(!fs.is_open()){
        throw std::runtime_error("Agents file does not exist");
    }

    std::string line;

    // nAgents line
    std::getline(fs, line);
    size_t nAgents = std::stoi(line);

    std::vector<AgentInfo> agentInfos;
    agentInfos.reserve(nAgents);

    for (int i = 0 ; i < nAgents; ++i){
        // row,col
        std::getline(fs, line);

        std::string colString, rowString;
        auto coordStream = std::stringstream(line);
        std::getline(coordStream, rowString, horizontalSep);
        std::getline(coordStream, colString, horizontalSep);

        agentInfos.push_back({dm.from2Dto1D(std::stoi(rowString), std::stoi(colString)), capacity, i});
    }

    return agentInfos;
}
