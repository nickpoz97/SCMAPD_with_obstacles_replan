//
// Created by nicco on 17/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP

#include "RePlanSimulator.hpp"
#include "WaitSimulator.hpp"
#include "Predictor.hpp"

class SmartSimulator : public RePlanSimulator, WaitSimulator{
public:
    SmartSimulator(const std::vector<RunningAgent>& runningAgents, const AmbientMap& ambientMap,
                   const nlohmann::json &obstaclesJson);

    void doSimulationStep(TimeStep t) override;
private:
    std::unordered_map<CompressedCoord, TimeStep> foundObstacles{};

    void applySmartChoice(const std::unordered_set<CompressedCoord> &allVisibleObstacles, TimeStep t);

    [[nodiscard]] int computeNoObsScore(int raId) const;
    [[nodiscard]] int computeObsScore(CompressedCoord obsPos, int raId) const;

    [[nodiscard]] int getScore(int raId, const vector<bool> &grid) const;

    std::unordered_set<CompressedCoord>
    updateAndGetNewObstacles(const std::unordered_set<CompressedCoord> &obstaclesPositions, TimeStep t);

    bool newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const;

    void applyRePlan(const std::unordered_set<CompressedCoord>& visibleObstacles);
};


#endif //SIMULTANEOUS_CMAPD_SMARTSIMULATOR_HPP
