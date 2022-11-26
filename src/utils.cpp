#include <utils.hpp>
#include <cnpy.h>
#include "Assignment.hpp"

DistanceMatrix utils::loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath) {
    const cnpy::NpyArray distanceMatrixObj = cnpy::npy_load(distanceMatrixPath.generic_string());

    unsigned startCoordsSize = distanceMatrixObj.shape[0] * distanceMatrixObj.shape[1];
    unsigned endCoordsSize = distanceMatrixObj.shape[2] * distanceMatrixObj.shape[3];

    // distance matrix is considered double
    const auto* data = distanceMatrixObj.data<double>();
    DistanceMatrix distanceMatrix(startCoordsSize, std::vector<CompressedCoord>(endCoordsSize));

    for (unsigned i = 0 ; i < startCoordsSize ; ++i){
        for (unsigned j = 0 ; j < endCoordsSize ; ++j){
            distanceMatrix[i][j] = static_cast<int>(data[i*startCoordsSize + j]);
        }
    }

    return distanceMatrix;
}

std::vector<Assignment>
utils::loadRobots(const std::filesystem::path &agentsFilePath, int& nCols, char horizontalSep, unsigned int capacity){
    std::ifstream fs (agentsFilePath, std::ios::in);
    std::string line;

    // nRows,nCols line
    std::getline(fs, line);

    std::string nRowsString;
    std::string nColsString;

    auto matrixSizeInfoStream = std::stringstream(line);
    std::getline(matrixSizeInfoStream, nRowsString, horizontalSep);
    std::getline(matrixSizeInfoStream, nColsString, horizontalSep);

    //nRows = std::stoi(nRowsString);
    nCols = std::stoi(nColsString);

    // nAgents line
    std::getline(fs, line);
    size_t nAgents = std::stoi(line);

    std::vector<Assignment> agents;
    agents.reserve(nAgents);

    for (unsigned i = 0 ; i < nAgents; ++i){
        std::getline(fs, line);

        std::string xCoordString, yCoordString;
        auto coordStream = std::stringstream(line);
        std::getline(coordStream, yCoordString, horizontalSep);
        std::getline(coordStream, xCoordString, horizontalSep);

        CompressedCoord cc = from2Dto1D(std::stoi(xCoordString), std::stoi(yCoordString), nCols);

        agents.emplace_back(cc, i, capacity);
    }

    return agents;
}

TasksVector utils::loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep){
    std::ifstream fs (tasksFilePath, std::ios::in);
    std::string line;

    // nTasks line
    std::getline(fs, line);
    size_t nTasks = std::stoi(line);

    TasksVector tasks;
    tasks.reserve(nTasks);

    for (unsigned i = 0 ; i < nTasks ; ++i){
        std::getline(fs, line);

        std::stringstream taskString{line};
        std::string value;

        std::getline(taskString, value, horizontalSep);
        int yBegin = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int xBegin = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int yEnd = std::stoi(value);

        std::getline(taskString, value, horizontalSep);
        int xEnd = std::stoi(value);

        unsigned releaseTime = std::getline(taskString, value, ',') ? std::stoi(value) : 0;

        tasks.push_back({from2Dto1D(xBegin, yBegin, nCols), from2Dto1D(xEnd, yEnd, nCols), releaseTime, i});
    }

    return tasks;
}

CompressedCoord utils::from2Dto1D(unsigned x, unsigned y, size_t nCols) {
    return y * static_cast<unsigned>(nCols) + x;
}
