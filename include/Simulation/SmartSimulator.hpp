//
// Created by nicco on 17/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP

#include "AbstractSimulator.hpp"
#include "Predictor.hpp"

struct PlanningResults{
    int score;
    std::vector<CompressedCoord> nextSteps;
};

class SmartSimulator : public AbstractSimulator{
public:
    SmartSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                   const nlohmann::json &obstaclesJson);

    void doSimulationStep(TimeStep t) override;
private:
    Predictor predictor;

    std::unordered_map<CompressedCoord, TimeStep> foundObstacles{};
    std::unordered_set<CompressedCoord> newObstacles{};

    // <raId, wait>
    [[nodiscard]] std::unordered_map<int, bool> getBestChoices(const SpawnedObstaclesSet &visibleObstacles) const;

    [[nodiscard]] int computeNoObsScore(int raId) const;
    [[nodiscard]] int computeObsScore(CompressedCoord obsPos, int raId) const;

    [[nodiscard]] int getScore(int raId, const vector<bool> &grid) const;

    std::unordered_set<CompressedCoord>
    getNewObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t);

    bool newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const;
};


#endif //SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
