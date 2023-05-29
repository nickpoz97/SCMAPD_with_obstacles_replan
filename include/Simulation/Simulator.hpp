//
// Created by nicco on 27/05/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_SIMULATOR_HPP


#include "Coord.hpp"
#include "RunningAgent.hpp"
#include "Instance.h"
#include "AmbientMap.hpp"

struct NormalInfo{
    TimeStep mu;
    TimeStep std;
};

enum class Strategy{
    RE_PLAN,
    WAIT,
    SMART
};

class Simulator {
public:
    Simulator(std::vector<RunningAgent> runningAgents, std::ifstream obstaclesCsv, AmbientMap ambientMap);
    void simulate(size_t hash, Strategy strategy);
    void printResults(const std::filesystem::path& out);
private:
    std::vector<RunningAgent> runningAgents;
    std::list<std::vector<CompressedCoord>> obstacles;
    std::unordered_map<CompressedCoord, NormalInfo> obstaclesTimeProb;
    AmbientMap ambientMap;

    Instance generatePBSInstance(const std::vector<CompressedCoord>& obstaclesLocation) const;
    void updatePlannedPaths(const std::vector<Path>& paths);

    std::vector<CompressedCoord> getNextPositions() const;

    bool rePlan(const std::vector<CompressedCoord>& actualObstacles);
    void wait(const std::vector<CompressedCoord>& actualObstacles);

    static std::list<std::vector<CompressedCoord>> getObstaclesFromCsv(std::ifstream obstaclesCsv);
};


#endif //SIMULTANEOUS_CMAPD_SIMULATOR_HPP
