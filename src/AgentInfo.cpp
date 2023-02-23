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

    std::vector<AgentInfo> agents;
    agents.reserve(nAgents);

    for (int i = 0 ; i < nAgents; ++i){
        std::getline(fs, line);

        std::string xCoordString, yCoordString;
        auto coordStream = std::stringstream(line);
        std::getline(coordStream, yCoordString, horizontalSep);
        std::getline(coordStream, xCoordString, horizontalSep);

        //CompressedCoord cc = DistanceMatrix::from2Dto1D(std::stoi(xCoordString), std::stoi(yCoordString), nCols);

        Coord position{std::stoi(yCoordString), std::stoi(xCoordString)};
        agents.push_back({dm.from2Dto1D(position), capacity, i});
    }

    return agents;
}