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
    Simulator(std::vector<RunningAgent> runningAgents, ObstaclesWrapper obstaclesWrapper, AmbientMap ambientMap);
    void simulate(Strategy strategy);
    void printResults(const std::filesystem::path &out, const nlohmann::json &sourceJson);
private:
    std::vector<RunningAgent> runningAgents;
    ObstaclesWrapper obstaclesWrapper;
    AmbientMap ambientMap;

    std::vector<Path> agentsHistory{};

    Instance
    generatePBSInstance(const SpawnedObstaclesSet &sOSet, TimeStep actualTimeStep,
                        FixedPaths fixedPaths, const std::unordered_set<int> &notAllowedAgents) const;
    void updatePlannedPaths(const std::vector<Path> &paths, const std::unordered_set<int> &waitingAgentsIds);

    std::vector<CompressedCoord> getNextPositions() const;

    bool rePlan(const SpawnedObstaclesSet &sOSet, TimeStep t);
    //bool wait(const std::vector<ObstaclePersistence>& obstaclesWithPermanence, TimeStep t, const std::unordered_set<int>& waitingAgents);

    static std::list<std::vector<CompressedCoord>> getObstaclesFromCsv(std::ifstream obstaclesCsv);

    bool solveWithPBS(const Instance &pbsInstance, const std::unordered_set<int> &waitingAgentsIds);

    vector<Path> extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const;

    FixedPaths extendsAndExtractFixedPaths(const std::unordered_set<int> &waitingAgents);

    std::unordered_set<int> getInvolvedAgents(const std::unordered_set<CompressedCoord>& actualObstacles) const;
};


#endif //SIMULTANEOUS_CMAPD_SIMULATOR_HPP
