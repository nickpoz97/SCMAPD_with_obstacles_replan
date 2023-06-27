//
// Created by nicco on 27/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_ABSTRACTPREDICTOBSTACLESWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_ABSTRACTPREDICTOBSTACLESWRAPPER_HPP

#include "AbstractObstaclesWrapper.hpp"

struct NormalInfo{
    int mu;
    int std;

    [[nodiscard]] double getProb(int interval) const;

    [[nodiscard]] inline auto getLow() const{
        return mu - 3 * std;
    }

    [[nodiscard]] inline auto getHigh() const{
        return mu + 3 * std;
    }
};

using ProbabilitiesMap = std::unordered_map<CompressedCoord, NormalInfo>;

class AbstractPredictObstaclesWrapper : public AbstractObstaclesWrapper {
public:
    AbstractPredictObstaclesWrapper(ProbabilitiesMap probabilitiesMap, ObstaclesMap obstaclesMap);

    std::unordered_map<TimeStep , double> getProbabilities(CompressedCoord obsPos) const;
protected:
    ProbabilitiesMap probabilitiesMap;

    static ProbabilitiesMap getProbabilitiesFromJson(const nlohmann::json &obstaclesJson);
};


#endif //SIMULTANEOUS_CMAPD_ABSTRACTPREDICTOBSTACLESWRAPPER_HPP
