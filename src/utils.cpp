#include <utils.hpp>
#include <fstream>
#include "Assignment.hpp"
#include "DistanceMatrix.hpp"

std::vector<Assignment>
utils::loadRobots(const std::filesystem::path &agentsFilePath, int nCols, char horizontalSep, unsigned int capacity){
    std::ifstream fs (agentsFilePath, std::ios::in);
    std::string line;

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

        //CompressedCoord cc = DistanceMatrix::from2Dto1D(std::stoi(xCoordString), std::stoi(yCoordString), nCols);

        Coord position{std::stoi(yCoordString), std::stoi(xCoordString)};
        agents.emplace_back(position, i, capacity);
    }

    return agents;
}

std::vector<Task> utils::loadTasks(const std::filesystem::path &tasksFilePath, int nCols, char horizontalSep){
    std::ifstream fs (tasksFilePath, std::ios::in);
    std::string line;

    // nTasks line
    std::getline(fs, line);
    size_t nTasks = std::stoi(line);

    std::vector<Task> tasks;
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


        tasks.push_back({{yBegin, xBegin}, {yEnd, xEnd}, releaseTime, i});
    }

    return tasks;
}
