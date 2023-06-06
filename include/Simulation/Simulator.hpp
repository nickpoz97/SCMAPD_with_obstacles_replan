//
// Created by nicco on 27/05/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_SIMULATOR_HPP


#include "Coord.hpp"
#include "RunningAgent.hpp"
#include "Instance.h"
#include "AmbientMap.hpp"
#include "ObstaclesWrapper.hpp"

enum class Strategy{
    RE_PLAN,
    WAIT,
    SMART
};

class Simulator {
public:
    Simulator(std::vector<RunningAgent> runningAgents, ObstaclesWrapper obstaclesWrapper, AmbientMap ambientMap,
              Strategy strategy);
    void simulate();
    void printResults(const std::filesystem::path &out, const nlohmann::json &sourceJson);
private:
    std::vector<RunningAgent> runningAgents;
    ObstaclesWrapper obstaclesWrapper;
    AmbientMap ambientMap;

    std::vector<Path> agentsHistory{};
    Strategy strategy;

    Instance
    generatePBSInstance(const SpawnedObstaclesSet &sOSet, const std::unordered_set<int> &waitingAgents) const;
    void updatePlannedPaths(const std::vector<Path> &paths);

    std::vector<CompressedCoord> getNextPositions() const;

    void rePlan(const SpawnedObstaclesSet &sOSet, TimeStep t);
    void wait(const SpawnedObstaclesSet &spawnedObstacles, const std::unordered_set<int> &waitingAgents);

    static std::list<std::vector<CompressedCoord>> getObstaclesFromCsv(std::ifstream obstaclesCsv);

    bool solveWithPBS(const Instance &pbsInstance);

    vector<Path> extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const;

    std::unordered_set<int>
    getInvolvedAgents(const SpawnedObstaclesSet &actualObstacles, TimeStep actualT) const;
};


#endif //SIMULTANEOUS_CMAPD_SIMULATOR_HPP
