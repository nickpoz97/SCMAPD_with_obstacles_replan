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
#include "AbstractObstaclesWrapper.hpp"

enum class Strategy{
    RE_PLAN,
    WAIT,
    SMART
};

class AbstractSimulator {
public:
    AbstractSimulator(
        std::vector<RunningAgent> runningAgents,
        AmbientMap ambientMap
    );


    void simulate();
    void printResults(const std::filesystem::path &out, const nlohmann::json &sourceJson);

    [[nodiscard]] vector<Path> getPaths() const;

    static std::vector<CompressedCoord> agentCPExtractor(const RunningAgent& ra, bool stopped);

    virtual ~AbstractSimulator() = default;
protected:
    std::vector<Path> agentsHistory{};

    AmbientMap ambientMap;
    std::vector<RunningAgent> runningAgents;

    std::unique_ptr<AbstractObstaclesWrapper> obsWrapper{};

    virtual void doSimulationStep(TimeStep t) = 0;

    [[nodiscard]] Instance
    generatePBSInstance(const std::unordered_set<CompressedCoord> &fixedObstacles,
        const SpawnedObstaclesSet &sOSet,
        const std::vector<std::vector<CompressedCoord>> &checkPoints
    ) const;

    static std::vector<Path> solveWithPBS(const Instance &pbsInstance);

    [[nodiscard]] std::vector<CompressedCoord> getNextPositions() const;

    void updatePlannedPaths(const std::vector<Path> &plannedPaths);

    [[nodiscard]] vector<Path> extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const;
    [[nodiscard]] vector<Path> extractPBSCheckpoints() const;

private:
    void updateHistory();
};


#endif //SIMULTANEOUS_CMAPD_ABSTRACTSIMULATOR_HPP
