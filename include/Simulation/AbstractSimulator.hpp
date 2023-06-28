//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_ABSTRACTSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_ABSTRACTSIMULATOR_HPP

#include <vector>
#include "Instance.h"
#include "SpawnedObstacle.hpp"
#include "RunningAgent.hpp"
#include "AmbientMap.hpp"
#include "ObstaclesWrapper.hpp"

enum class Strategy{
    RE_PLAN,
    WAIT,
    SMART
};

class AbstractSimulator {
public:
    AbstractSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
        const nlohmann::json& obstaclesJson);

    void simulate();
    void printResults(const std::filesystem::path &out, const nlohmann::json &sourceJson);

    [[nodiscard]] vector<Path> getPaths() const;

    virtual ~AbstractSimulator() = default;
protected:
    std::vector<Path> agentsHistory{};

    AmbientMap ambientMap;
    std::vector<RunningAgent> runningAgents;

    ObstaclesWrapper obsWrapper;

    virtual void doSimulationStep(TimeStep t) = 0;

    [[nodiscard]] Instance
    generatePBSInstance(const std::unordered_set<CompressedCoord> &fixedObstacles,
        const SpawnedObstaclesSet &sOSet,
        const std::vector<std::vector<CompressedCoord>> &checkPoints
    ) const;

    std::vector<Path> solveWithPBS(const Instance &pbsInstance, const std::unordered_set<int> &excludedAgentsIds) const;
    std::vector<Path> solveWithPBS(const Instance &pbsInstance) const;

    [[nodiscard]] std::vector<CompressedCoord> getNextPositions() const;

    void updatePlannedPaths(const std::vector<Path> &plannedPaths);

    [[nodiscard]] vector<Path> extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const;
    [[nodiscard]] vector<Path> extractPBSCheckpoints() const;

    static vector<CompressedCoord> getExtendedCheckpoints(const RunningAgent &ra);
private:
    void updateHistory();
};


#endif //SIMULTANEOUS_CMAPD_ABSTRACTSIMULATOR_HPP
