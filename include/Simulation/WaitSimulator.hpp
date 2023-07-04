//
// Created by nicco on 13/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP
#define SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP

#include "AbstractSimulator.hpp"

class WaitSimulator : virtual public AbstractSimulator{
public:
    WaitSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap, const nlohmann::json &obstaclesJson);
protected:
    void doSimulationStep(TimeStep t) override;
    bool needRePlan = false;

    // obstacles
    using ObsAgentsMap = std::unordered_map<CompressedCoord, std::unordered_set<int>>;
    ObsAgentsMap obsAgentsMap{};
    std::unordered_set<CompressedCoord> waitingAgentsPos;

    void setRePlan(int formerObstaclePos);

    [[nodiscard]] Instance
    generatePBSInstance(const std::unordered_set<CompressedCoord> &fixedObstacles,
                        const std::vector<std::vector<CompressedCoord>> &checkPoints
    ) const;

    void wait(int waitingAgentIndex, int obstaclePos);
    std::unordered_set<int> getWaitingAgentsIds() const;

    void chooseStatusForAgents(const std::vector<CompressedCoord> &nextPositions,
                               const std::unordered_set<CompressedCoord> &visibleObstacles);

    std::unordered_set<CompressedCoord>
    getExtendedObstacles(const std::unordered_set<CompressedCoord> &visibleObstacles) const;

    std::unordered_set<CompressedCoord> getWaitingAgentsPositions() const;
private:

    void rePlanFreeAgents();
};


#endif //SIMULTANEOUS_CMAPD_WAITSIMULATOR_HPP
