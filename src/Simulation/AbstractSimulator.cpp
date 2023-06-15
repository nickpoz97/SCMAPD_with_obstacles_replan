//
// Created by nicco on 13/06/2023.
//

#include "Simulation/AbstractSimulator.hpp"
#include "PBS.h"

void AbstractSimulator::simulate(){
    using std::ranges::any_of;

    for(TimeStep t = 0 ; any_of(runningAgents, [](const RunningAgent& ra){return !ra.hasFinished();}) ; ++t) {
        updateHistory();

        doSimulationStep(t);

        //update agents
        std::ranges::for_each(runningAgents, [](RunningAgent& ra){ra.stepAndUpdate();});
    }
    // last position
    updateHistory();
}

void AbstractSimulator::updateHistory() {
    for (const auto &ra: runningAgents) {
        agentsHistory[ra.getAgentId()].push_back(ra.getPlannedPath().front());
    }
}

Instance AbstractSimulator::generatePBSInstance(const SpawnedObstaclesSet &sOSet,
                                                const std::unordered_set<int> &stoppedAgents) const {
    vector<Path> agentsCheckpoints = extractPBSCheckpoints(stoppedAgents);

    auto grid{ambientMap.getGrid()};

    // waiting agents are like walls
    for (CompressedCoord aId : stoppedAgents){
        assert(agentsCheckpoints[aId].size() == 1);
        grid[runningAgents[aId].getActualPosition()] = true;
    }

    return {
        std::move(grid),
        agentsCheckpoints,
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        sOSet
    };
}

vector<Path> AbstractSimulator::extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const {
    auto checkPointsExtractor = [&notAllowedAgents](const RunningAgent& ra) -> std::vector<CompressedCoord>{
        // the first one is the first position
        std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

        // not moving
        if (!notAllowedAgents.contains(ra.getAgentId())){
            std::ranges::copy(
                    ra.getPlannedCheckpoints(),
                    std::back_inserter(checkPoints)
            );
        }

        return checkPoints;
    };

    // take out waiting agents and extract the checkpoints of the remaining ones
    auto agentsCheckpoints = runningAgents | std::views::transform(checkPointsExtractor);
    return {agentsCheckpoints.begin(), agentsCheckpoints.end()};
}

Instance AbstractSimulator::generatePBSInstance(const SpawnedObstaclesSet &sOSet) const {
    return generatePBSInstance(sOSet, {});
}

Instance AbstractSimulator::generatePBSInstance() const {
    return generatePBSInstance({}, {});
}

std::vector<Path> AbstractSimulator::solveWithPBS(const Instance &pbsInstance) {
    PBS pbs{pbsInstance, true, 0};
    if(pbs.solve(7200)){
        return pbs.getPaths();
    }
    throw std::runtime_error("No Path");
}

AbstractSimulator::AbstractSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap) :
    agentsHistory{runningAgents.size(), Path{}},
    runningAgents{std::move(runningAgents)},
    ambientMap{std::move(ambientMap)}
{}

std::vector<CompressedCoord> AbstractSimulator::getNextPositions() const {
    auto nextPositions =
        runningAgents |
        std::views::transform( [](const auto& ra){return ra.getNextPosition();} );

    return {nextPositions.begin(), nextPositions.end()};
}

void AbstractSimulator::updatePlannedPaths(const std::vector<Path> &plannedPaths) {
    auto pathsIt = plannedPaths.cbegin();

    for (auto& actualAgent : runningAgents){
        int agentId = actualAgent.getAgentId();
        actualAgent.setPlannedPath(plannedPaths[agentId]);
        //assert(actualAgent.checkpointChecker());
    }
}

void AbstractSimulator::printResults(const std::filesystem::path &out, const nlohmann::json &sourceJson) {
    using namespace nlohmann;

    json j;
    j["agents"] = json::array();

    int i = 0;
    for (const auto& ah : agentsHistory){
        auto waypoints = json::array();
        for(const auto& wp : sourceJson["agents"][i++]["waypoints"]){
            waypoints.push_back({
                                        {"coords", wp["coords"]},
                                        {"demand", wp["demand"]}
                                });
        }
        j["agents"].push_back({
                                      {"path", static_cast<json>(getVerbosePath(ah, ambientMap.getNCols()))},
                                      {"waypoints", waypoints}
                              });
    }

    std::ofstream file(out);
    file << j.dump();
}

size_t AbstractSimulator::getResultPenalty(bool useMakespan, const vector<Path> &paths) {
    size_t value = useMakespan ?
                   std::ranges::max_element(paths, [](const Path& a, const Path& b){return a.size() < b.size();})->size() :
                   std::accumulate(paths.cbegin(), paths.cend(), 0, [](size_t sum, const Path& path){return sum + path.size();});
    return value;
}

Interval AbstractSimulator::getScore(const std::vector<CompressedCoord> &obstaclesPositions, bool useMakespan) const {
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
            auto paths = solveWithPBS(generatePBSInstance(sOSet, {}));
            auto value = getResultPenalty(useMakespan, paths);

            // weighting with p
            score += static_cast<double>(value) * p;
        }
    }

    assert(!obstaclesPositions.empty());
    // scaling with number of obstacles (obstacles appearance p is constant)
    score /= static_cast<double>(obstaclesPositions.size());

    auto idealPaths = solveWithPBS(generatePBSInstance({}, {}));
    return static_cast<Interval>(std::ceil(score)) - getResultPenalty(useMakespan, idealPaths);
}