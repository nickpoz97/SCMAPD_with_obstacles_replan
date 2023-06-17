//
// Created by nicco on 17/06/2023.
//

#include "Simulation/SmartSimulator.hpp"
#include "Simulation/SmartObstaclesWrapper.hpp"
#include "Simulation/WaitSimulator.hpp"

SmartSimulator::SmartSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                               const nlohmann::json &obstaclesJson,
                               bool useMakeSpan) :
        AbstractSimulator(std::move(runningAgents), std::move(ambientMap)),
        useMakeSpan{useMakeSpan}
{
    obsWrapper = std::make_unique<SmartObstaclesWrapper>(obstaclesJson);
}

size_t SmartSimulator::getResultPenalty(const vector<Path> &paths) const{
    size_t value = useMakeSpan ?
        std::ranges::max_element(paths, [](const Path& a, const Path& b){return a.size() < b.size();})->size() :
        std::accumulate(paths.cbegin(), paths.cend(), 0, [](size_t sum, const Path& path){return sum + path.size();});
    return value;
}

int SmartSimulator::getScore(const std::vector<CompressedCoord> &obstaclesPositions, bool useMakespan) const {
    if(obstaclesPositions.empty()){
        return 0;
    }
    double score = 0;

    for(const auto pos : obstaclesPositions){

        for(const auto [permanence, p] : obsWrapper->getProbabilities(pos)){
            SpawnedObstaclesSet sOSet;
            for(Interval i = 1 ; i <= permanence ; ++i){
                sOSet.emplace(i, pos);
            }
            // not using waiting agents
            auto paths = solveWithPBS(generatePBSInstance(extractPBSCheckpoints({})));
            auto value = getResultPenalty(paths);

            // weighting with p
            score += static_cast<double>(value) * p;
        }
    }

    assert(!obstaclesPositions.empty());
    // scaling with number of obstacles (obstacles appearance p is constant)
    score /= static_cast<double>(obstaclesPositions.size());

    auto idealPaths = solveWithPBS(generatePBSInstance(extractPBSCheckpoints()));
    return static_cast<Interval>(std::ceil(score)) - getResultPenalty(idealPaths);
}

PlanningResults SmartSimulator::simulateWithWait(const ObstaclesMap &obstaclesMap) const {
    // todo make this
    //WaitSimulator ws{};
}
