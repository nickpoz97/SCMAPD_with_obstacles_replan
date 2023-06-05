//
// Created by nicco on 27/05/2023.
//

#include <fstream>
#include <boost/tokenizer.hpp>
#include "Simulation/Simulator.hpp"
#include "Simulation/PP.hpp"

void Simulator::simulate() {
    using std::ranges::any_of;

    for(TimeStep t = 0 ; any_of(runningAgents, [](const RunningAgent& ra){return !ra.hasFinished();}) ; ++t){
        for(const auto& ra : runningAgents){
            agentsHistory[ra.getAgentId()].push_back(ra.getPlannedPath().front());
        }

        // extract obstacles at this timeStep (empty forward list otherwise)
        const auto& actualObstacles = obstaclesWrapper.updateAndGet(t, getNextPositions());

        if(!actualObstacles.empty()){
            const auto involvedAgents{getInvolvedAgents(actualObstacles, t)};

            switch (strategy) {
                // warning cannot solve if obstacles is on a target
                case Strategy::RE_PLAN:
                    rePlan(actualObstacles);
                break;
                case Strategy::WAIT:
                    wait(
                            actualObstacles,
                            involvedAgents
                    );
                break;
                default:
                    throw std::runtime_error("Not handled case");
                break;
            }
        }

        std::ranges::for_each(runningAgents, [](RunningAgent& ra){ra.stepAndUpdate();});
    }
    for(const auto& ra : runningAgents){
        agentsHistory[ra.getAgentId()].push_back(ra.getPlannedPath().front());
    }
}

std::vector<CompressedCoord> Simulator::getNextPositions() const {

    auto nextPositions =
        runningAgents |
        std::views::transform( [](const auto& ra){return ra.getNextPosition();} ) |
        std::views::filter( [](const auto& optCoord){return optCoord.has_value();} ) |
        std::views::transform( [](const auto& coord){return *coord;} );

    return {nextPositions.begin(), nextPositions.end()};
}

std::vector<std::vector<CompressedCoord>> Simulator::extractCheckpoints(const std::unordered_set<int> &notAllowedAgents) const {
    auto checkPointsExtractor = [&notAllowedAgents](const RunningAgent& ra) -> std::vector<CompressedCoord>{
        // the first one is the first position
        std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

        // check if not waiting
        if (!notAllowedAgents.contains(ra.getAgentId())){
            std::ranges::copy(
                ra.getPlannedCheckpoints(),
                std::back_inserter(checkPoints)
            );
        }

        return checkPoints;
    };

    auto agentsCheckpoints = runningAgents |
        std::views::transform(checkPointsExtractor);
    return {agentsCheckpoints.begin(), agentsCheckpoints.end()};
}

void Simulator::updatePlannedPaths(const std::vector<Path> &paths) {
    auto pathsIt = paths.cbegin();

    for (auto& actualAgent : runningAgents){
        actualAgent.setPlannedPath(paths[actualAgent.getAgentId()]);
        //assert(actualAgent.checkpointChecker());
    }
}

bool Simulator::rePlan(const SpawnedObstaclesSet &sOSet) {
    updatePlannedPaths(PathFinder::solveWithPP(sOSet, extractCheckpoints({}), ambientMap));
    return true;
}

Simulator::Simulator(std::vector<RunningAgent> runningAgents, ObstaclesWrapper obstaclesWrapper, AmbientMap ambientMap,
                     Strategy strategy) :
    runningAgents{std::move(runningAgents)},
    obstaclesWrapper(std::move(obstaclesWrapper)),
    ambientMap{std::move(ambientMap)},
    agentsHistory{this->runningAgents.size()},
    strategy{strategy}
{}

void Simulator::printResults(const std::filesystem::path &out, const nlohmann::json &sourceJson) {
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

bool Simulator::wait(const SpawnedObstaclesSet &sOSet, const std::unordered_set<int> &waitingAgents) {
    updatePlannedPaths(PathFinder::solveWithPP(sOSet, extractCheckpoints(waitingAgents), ambientMap));
    return true;
}

std::unordered_set<int>
Simulator::getInvolvedAgents(const SpawnedObstaclesSet &actualObstacles, TimeStep actualT) const {
    auto result = runningAgents |
        std::views::filter(
            [&actualObstacles, actualT](const RunningAgent& ra){
                auto nextPos = ra.getNextPosition();
                return nextPos.has_value() && actualObstacles.contains({1, *nextPos});
            }
        ) |
        std::views::transform([](const RunningAgent& ra){return ra.getAgentId();});

    return {result.begin(), result.end()};
}
