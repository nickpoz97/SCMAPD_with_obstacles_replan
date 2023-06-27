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
    void doSimulationStep(TimeStep t) override;
    bool rePlan = false;

    // obstacles
    using ObsAgentsMap = std::unordered_map<CompressedCoord, std::unordered_set<int>>;
    ObsAgentsMap obsAgentsMap{};
    std::unordered_set<CompressedCoord> noCrossPositions;

    void setRePlan(int formerObstaclePos);

    void extendWaitingPositions(const std::unordered_map<int, CompressedCoord> &wAgentsNextPos);

    [[nodiscard]] Instance
    generatePBSInstance(const std::unordered_set<CompressedCoord> &fixedObstacles,
                        const std::vector<std::vector<CompressedCoord>> &checkPoints
    ) const;

    void extendWaitingPositions();
    void wait(int waitingAgentIndex, int obstaclePos);
    std::unordered_set<int> getWaitingAgentsIds() const;

    void rePlanFreeAgents();

    void chooseStatusForAgents(const vector<CompressedCoord> &nextPositions,
                               const std::unordered_set<CompressedCoord> &actualObstacles);
};


#endif //SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP
