//
// Created by nicco on 17/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SMARTOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_SMARTOBSTACLESWRAPPER_HPP

#include "AbstractObstaclesWrapper.hpp"

class SmartObstaclesWrapper : public AbstractObstaclesWrapper{
public:
    explicit SmartObstaclesWrapper(const nlohmann::json &obstaclesJson);
    void update(TimeStep actualT, const std::vector<CompressedCoord> &nextPositions) override;
    SpawnedObstaclesSet get() const override;

private:
    std::unordered_map<CompressedCoord, TimeStep> foundObstacles{};
    std::unordered_set<CompressedCoord> newObstacles{};

    std::unordered_set<CompressedCoord>
    updateFoundObstacles(const std::vector<CompressedCoord> &obstaclesPositions, TimeStep t);

    bool newAppearance(CompressedCoord pos, TimeStep firstSpawnTime, TimeStep actualSpawnTime) const;
};


#endif //SIMULTANEOUS_CMAPD_SMARTOBSTACLESWRAPPER_HPP
