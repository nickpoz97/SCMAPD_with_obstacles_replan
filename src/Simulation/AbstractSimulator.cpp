//
// Created by nicco on 13/06/2023.
//

#include "Simulation/AbstractSimulator.hpp"
#include "PBS.h"

void AbstractSimulator::simulate(){
    using std::ranges::any_of;

    auto startT = std::chrono::steady_clock::now();
    for(TimeStep t = 0 ; any_of(runningAgents, [](const RunningAgent& ra){return !ra.hasFinished();}) ; ++t) {
        updateHistory();

        doSimulationStep(t);

        //update agents
        std::ranges::for_each(runningAgents, [t](RunningAgent& ra){ ra.stepAndUpdate(t);});
    }
    // last position
    updateHistory();
    executionTime =
        std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - startT).count();
}

void AbstractSimulator::updateHistory() {
    for (const auto &ra: runningAgents) {
        agentsHistory[ra.getAgentId()].push_back(ra.getPlannedPath().front());
    }
}

std::vector<CompressedCoord> AbstractSimulator::getExtendedCheckpoints(const RunningAgent &ra) {
    std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

    std::ranges::copy(
        ra.getPlannedCheckpoints(),
        std::back_inserter(checkPoints)
    );

    return checkPoints;
};

std::vector<Path> AbstractSimulator::extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const {
    // take out waiting agents and extract the checkpoints of the remaining ones
    auto agentsCheckpoints = runningAgents |
        std::views::filter([&notAllowedAgents](const RunningAgent& ra){return !notAllowedAgents.contains(ra.getAgentId());}) |
        std::views::transform([](const auto& ra) {return getExtendedCheckpoints(ra);});
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

std::unordered_map<int, Path>
AbstractSimulator::solveWithPBS(const Instance &pbsInstance, const std::unordered_set<int> &excludedAgentsIds) const{
    PBS pbs{pbsInstance, true, 0};
    if(!pbs.solve(7200)) {
        throw std::runtime_error("No Path");
    }

    auto nAgents = runningAgents.size();
    auto computedPaths = pbs.getPaths();
    std::unordered_map<int, Path> indexedPaths;

    assert(nAgents == computedPaths.size() + excludedAgentsIds.size());

    auto cPIt = computedPaths.cbegin();
    for(int raId = 0 ; raId < nAgents ; ++raId){
        if(!excludedAgentsIds.contains(raId)){
            indexedPaths[raId] = *cPIt++;
        }
    }

    return indexedPaths;
}

std::unordered_map<int, Path>
AbstractSimulator::solveWithPBS(const Instance &pbsInstance) const{
    return solveWithPBS(pbsInstance, {});
}

AbstractSimulator::AbstractSimulator(std::vector<RunningAgent> runningAgents, AmbientMap ambientMap,
                                     const nlohmann::json& obstaclesJson) :
    agentsHistory{runningAgents.size(), Path{}},
    runningAgents{std::move(runningAgents)},
    ambientMap{std::move(ambientMap)},
    obsWrapper{obstaclesJson}
{}

std::vector<CompressedCoord> AbstractSimulator::getNextPositions() const {
    auto nextPositions =
        runningAgents |
        std::views::transform( [](const auto& ra){return ra.getNextPosition();} );

    return {nextPositions.begin(), nextPositions.end()};
}

void AbstractSimulator::updatePlannedPaths(const std::unordered_map<int, Path> &plannedPaths) {
    for (auto& [raId, newPath] : plannedPaths){
        runningAgents[raId].setPlannedPath(newPath);
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
    j["stats"] = {
        {"makespan", compute_makespan()},
        {"total_travel_distance", compute_ttd()},
        {"total_travel_time", compute_ttt()},
        {"time", executionTime}
    };

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

TimeStep AbstractSimulator::compute_ttd() const {
    return std::accumulate(
        agentsHistory.cbegin(),
        agentsHistory.cend(),
        0,
        [](TimeStep acc, const Path& p){return acc + compute_cumulated_distance(p);}
    );
}

TimeStep AbstractSimulator::compute_cumulated_distance(const Path& path) {
    std::optional<CompressedCoord> former{std::nullopt};
    TimeStep cumulatedDistance = 0;

    for (auto pos: path) {
        // agent not moving
        if (former.has_value() && former.value() == pos) {
            continue;
        }
        ++cumulatedDistance;
        former = pos;
    }

    return cumulatedDistance;
}

TimeStep AbstractSimulator::compute_ttt() const {
    return std::accumulate(
        runningAgents.cbegin(),
        runningAgents.cend(),
        0,
        [](TimeStep acc, const RunningAgent& ra){return acc + ra.getArrivalTimeStep();}
    );
}

TimeStep AbstractSimulator::compute_makespan() const {
    return static_cast<TimeStep>(std::ranges::max_element(
        agentsHistory,
        [](const Path& p1, const Path& p2){return p1.size() < p2.size();}
    )->size());
}
