#include <utils.hpp>
#include <cnpy.h>
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

        CompressedCoord cc = from2Dto1D(std::stoi(xCoordString), std::stoi(yCoordString), nCols);

        agents.emplace_back(cc, i, capacity);
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

        tasks.push_back({from2Dto1D(xBegin, yBegin, nCols), from2Dto1D(xEnd, yEnd, nCols), releaseTime, i});
    }

    return tasks;
}

namespace cmapd{

std::vector<std::pair<Point, Point>> taskToPointVec(const std::vector<Task>& v, size_t nCols){
    std::vector<std::pair<Point, Point>> newV{};
    newV.reserve(v.size());

    for (const Task& t : v){
        newV.emplace_back(
            Point(t.startLoc / nCols, t.startLoc % nCols),
            Point(t.goalLoc / nCols, t.goalLoc % nCols)
        );
    }

    return newV;
}

std::vector<Point> robotToPointVec(const std::vector<Assignment>& v, size_t nCols){
    std::vector<Point> newV;
    newV.reserve(v.size());

    for(const auto& a : v){
        newV.emplace_back(a.getStartPosition() / nCols, a.getStartPosition() % nCols);
    }

    return newV;
}

}
