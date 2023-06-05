//
// Created by nicco on 27/05/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_SIMULATOR_HPP


#include "Coord.hpp"
#include "RunningAgent.hpp"
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

    void updatePlannedPaths(const std::vector<Path> &paths);

    std::vector<CompressedCoord> getNextPositions() const;

    bool rePlan(const SpawnedObstaclesSet &sOSet);
    bool wait(const SpawnedObstaclesSet &sOSet, const std::unordered_set<int> &waitingAgents);

    std::vector<std::vector<CompressedCoord>> extractCheckpoints(const std::unordered_set<int> &notAllowedAgents) const;

    std::unordered_set<int>
    getInvolvedAgents(const SpawnedObstaclesSet &actualObstacles, TimeStep actualT) const;
};


#endif //SIMULTANEOUS_CMAPD_SIMULATOR_HPP
