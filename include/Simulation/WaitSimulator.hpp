//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP

#include "AbstractSimulator.hpp"

class WaitSimulator : public AbstractSimulator{
public:
    WaitSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson);
private:
    // obstacles
    using ObsAgentsMap = std::unordered_map<CompressedCoord, std::unordered_set<int>>;
    ObsAgentsMap obsAgentsMap{};
    std::unordered_set<CompressedCoord> noCrossPositions;

    void doSimulationStep(TimeStep t) override;

    std::unordered_set<int> getWaitingAgentsIds() const;

    void wait(int waitingAgentIndex, int obstaclePos);

    void rePlan(int freeAgentId, int formerObstaclePos);

    void extendWaitingPositions();
};


#endif //SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP
