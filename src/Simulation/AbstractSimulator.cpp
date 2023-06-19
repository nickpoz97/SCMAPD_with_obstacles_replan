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

std::vector<CompressedCoord> AbstractSimulator::agentCPExtractor(const RunningAgent& ra, bool stopped){
    std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

    // blocked in starting position
    if (stopped){
        return checkPoints;
    }

    std::ranges::copy(
        ra.getPlannedCheckpoints(),
        std::back_inserter(checkPoints)
    );

    return checkPoints;
}

vector<Path> AbstractSimulator::extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const {

    // take out waiting agents and extract the checkpoints of the remaining ones
    auto agentsCheckpoints = runningAgents | std::views::transform([&notAllowedAgents](const auto& ra) {
        return agentCPExtractor(ra, notAllowedAgents.contains(ra.getAgentId()));
    });
    return {agentsCheckpoints.begin(), agentsCheckpoints.end()};
}

Instance AbstractSimulator::generatePBSInstance(const std::unordered_set<CompressedCoord> &fixedObstacles,
    const SpawnedObstaclesSet &sOSet,
    const vector<std::vector<CompressedCoord>> &checkPoints) const
{
    auto grid{ambientMap.getGrid()};

    // fixed obstacles are like walls
    for (CompressedCoord cc : fixedObstacles){
        grid[cc] = true;
    }

    return {
        std::move(grid),
        checkPoints,
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        sOSet
    };
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

vector<Path> AbstractSimulator::extractPBSCheckpoints() const {
    return extractPBSCheckpoints({});
}

vector<Path> AbstractSimulator::getPaths() const {
    decltype(AbstractSimulator::getPaths()) ret{};

    std::ranges::transform(
        runningAgents,
        std::back_inserter(ret),
        [](const RunningAgent& ra){return ra.getPlannedPath();}
    );

    return ret;
}
