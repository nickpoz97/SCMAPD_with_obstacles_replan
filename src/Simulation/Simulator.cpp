//
// Created by nicco on 27/05/2023.
//

#include <boost/tokenizer.hpp>
#include "Simulation/Simulator.hpp"
#include "PBS.h"

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
                    rePlan(actualObstacles, t);
                break;
                case Strategy::WAIT:
                    wait(
                        actualObstacles,
                        t,
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

Instance
Simulator::generatePBSInstance(const SpawnedObstaclesSet &sOSet, const std::unordered_set<int> &waitingAgents) const {
    vector<Path> agentsCheckpoints = extractPBSCheckpoints(waitingAgents);

    return {
        ambientMap.getGrid(),
        agentsCheckpoints,
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        sOSet
    };
}

vector<Path> Simulator::extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const {
    auto checkPointsExtractor = [&notAllowedAgents](const RunningAgent& ra) -> std::vector<CompressedCoord>{
        // the first one is the first position
        std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

        if (!notAllowedAgents.contains(ra.getAgentId())){
            std::ranges::copy(
                ra.getPlannedCheckpoints(),
                std::back_inserter(checkPoints)
            );
        }

        return checkPoints;
    };

    // take out waiting agents and extract the checkpoints of the remaining ones
    auto agentsCheckpoints = runningAgents |
        std::views::transform(checkPointsExtractor);
    return {agentsCheckpoints.begin(), agentsCheckpoints.end()};
}

void Simulator::updatePlannedPaths(const std::vector<Path> &paths) {
    auto pathsIt = paths.cbegin();

    for (auto& actualAgent : runningAgents){
        actualAgent.setPlannedPath(paths[actualAgent.getAgentId()]);
        assert(actualAgent.checkpointChecker());
    }
}

bool Simulator::rePlan(const SpawnedObstaclesSet& sOSet , TimeStep t) {
    auto pbsInstance{generatePBSInstance(sOSet, {})};
    return solveWithPBS(pbsInstance);
}

bool Simulator::solveWithPBS(const Instance &pbsInstance) {
    PBS pbs{pbsInstance, true, 0};
    if(pbs.solve(7200)){
        updatePlannedPaths(pbs.getPaths());
        return true;
    }
    return false;
}

std::list<std::vector<CompressedCoord>> Simulator::getObstaclesFromCsv(std::ifstream obstaclesCsv) {
    using Tokenizer = boost::tokenizer<boost::escaped_list_separator<char>>;

    std::list<std::vector<CompressedCoord>> obstaclesList{};

    std::string line;
    std::getline(obstaclesCsv, line);

    Tokenizer tok(line);
    if(*tok.begin() != "obs_0"){
        throw std::runtime_error("Wrong csv file");
    }

    while(std::getline(obstaclesCsv, line)){
        tok = line;

        auto actualObstacles = tok |
        std::views::filter([](const std::string& token){return token != "-1";}) |
        std::views::transform([](const std::string& token){return std::stoi(token);});

        obstaclesList.emplace_back(actualObstacles.begin(), actualObstacles.end());
    }

    return obstaclesList;
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

bool Simulator::wait(const SpawnedObstaclesSet & spawnedObstacles, TimeStep t, const std::unordered_set<int>& waitingAgents) {
    auto pbsInstance{
        generatePBSInstance(spawnedObstacles, waitingAgents)
    };
    return solveWithPBS(pbsInstance);
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
