//
// Created by nicco on 27/05/2023.
//

#include <random>
#include <boost/tokenizer.hpp>
#include "Simulation/Simulator.hpp"
#include "PBS.h"

void Simulator::simulate(Strategy strategy) {
    using std::ranges::any_of;

    auto gen = std::default_random_engine(computeSeed());

    for(TimeStep t = 0 ; any_of(runningAgents, [](const RunningAgent& ra){return !ra.hasFinished();}) ; ++t){
        // extract obstacles at this timeStep (empty forward list otherwise)
        // agents don' t know about this vector
        const auto& actualObstacles = obstacles[t];

        for(const auto& ra : runningAgents){
            agentsHistory[ra.getAgentId()].push_back(ra.getPlannedPath().front());
        }

        const auto involvedAgents{getInvolvedAgents(actualObstacles)};


        auto obstaclesWithPermanence = getNextPositions() |
            std::views::filter([&](CompressedCoord cc){return actualObstacles.contains(cc);}) |
            std::views::transform([&](CompressedCoord cc) -> ObstaclePersistence{
                auto [mu, std] = obstaclesTimeProb[cc];
                std::normal_distribution<float> d(static_cast<float>(mu), static_cast<float>(std));
                return {cc, static_cast<Interval>(d(gen))};}
            );

        if(!obstaclesWithPermanence.empty()){
            switch (strategy) {
                // warning cannot solve if obstacles is on a target
                case Strategy::RE_PLAN:
                    rePlan({obstaclesWithPermanence.begin(), obstaclesWithPermanence.end()}, t);
                break;
                case Strategy::WAIT:
                    wait(
                        {obstaclesWithPermanence.begin(), obstaclesWithPermanence.end()},
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
Simulator::generatePBSInstance(const std::vector<ObstaclePersistence> &obstaclesWithPermanence, TimeStep actualTimeStep,
                               FixedPaths fixedPaths, const std::unordered_set<int> &notAllowedAgents) const {
    vector<Path> agentsCheckpoints = extractPBSCheckpoints(notAllowedAgents);

    // extract grid
    auto grid{ambientMap.getGrid()};
    assert(grid.size() == ambientMap.getNRows() * ambientMap.getNCols());

    return {
        grid,
        agentsCheckpoints,
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        getSpawnedObstacles(obstaclesWithPermanence),
        std::move(fixedPaths)
    };
}

vector<Path> Simulator::extractPBSCheckpoints(const std::unordered_set<int> &notAllowedAgents) const {
    auto checkPointsExtractor = [](const RunningAgent& ra) -> std::vector<CompressedCoord>{
        // the first one is the first position
        std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

        std::ranges::copy(
            ra.getPlannedCheckpoints(),
            std::back_inserter(checkPoints)
        );

        return checkPoints;
    };

    // take out waiting agents and extract the checkpoints of the remaining ones
    auto agentsCheckpoints = runningAgents |
        std::views::filter([&](const RunningAgent& ra){return !notAllowedAgents.contains(ra.getAgentId());}) |
        std::views::transform(checkPointsExtractor);

    assert(std::ranges::distance(agentsCheckpoints) == (runningAgents.size() - notAllowedAgents.size()));
    return {agentsCheckpoints.begin(), agentsCheckpoints.end()};
}

SpawnedObstaclesSet
Simulator::getSpawnedObstacles(const vector<ObstaclePersistence> &obstaclesWithPermanence) {
    SpawnedObstaclesSet spawnedObstaclesSet{};
    for (const auto& owp : obstaclesWithPermanence){
        for(int t = 0 ; t < owp.duration ; ++t){
            spawnedObstaclesSet.emplace(t, owp.loc);
        }
    }
    return spawnedObstaclesSet;
}

void Simulator::updatePlannedPaths(const std::vector<Path> &paths, const std::unordered_set<int> &waitingAgentsIds) {
    auto pathsIt = paths.cbegin();

    auto notWaitingAgents = runningAgents |
        std::views::transform([](const RunningAgent& ra){return ra.getAgentId();}) |
        std::views::filter([&waitingAgentsIds](int agentId){return !waitingAgentsIds.contains(agentId);});

    for (int agentId : notWaitingAgents){
        auto& actualAgent = runningAgents[agentId];

        actualAgent.setPlannedPath(*pathsIt);
        ++pathsIt;
        assert(actualAgent.checkpointChecker());
    }
}

bool Simulator::rePlan(const std::vector<ObstaclePersistence>& obstaclesWithPermanence, TimeStep t) {
    auto pbsInstance{generatePBSInstance(obstaclesWithPermanence, t, {}, std::unordered_set<int>())};
    return solveWithPBS(pbsInstance, {});
}

bool Simulator::solveWithPBS(const Instance &pbsInstance, const std::unordered_set<int> &waitingAgentsIds) {
    PBS pbs{pbsInstance, true, 0};
    if(pbs.solve(7200)){
        updatePlannedPaths(pbs.getPaths(), waitingAgentsIds);
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

ObstaclesMap Simulator::getObstaclesFromJson(const nlohmann::json &obstaclesJson) {
    ObstaclesMap obstacles{};

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        TimeStep from = obsObj["t"];
        TimeStep to = from + static_cast<Interval>(obsObj["interval"]);

        for(auto t = from ; t < to ; ++t){
            obstacles[t].insert(static_cast<CompressedCoord>(obsObj["pos"]));
        }
    }

    return obstacles;
}

ProbabilitiesMap Simulator::getProbabilitiesFromJson(const nlohmann::json &obstaclesJson) {
    ProbabilitiesMap probabilitiesMap{};

    // one distribution for each obstacle
    const auto& pObj = obstaclesJson["probability"];

    for(const auto& obsObj : obstaclesJson["obstacles"]){
        probabilitiesMap[obsObj["pos"]] = NormalInfo{pObj["mu"], pObj["sigma"]};
    }

    return probabilitiesMap;
}

Simulator::Simulator(std::vector<RunningAgent> runningAgents, const nlohmann::json &obstaclesJson, AmbientMap ambientMap) :
    runningAgents{std::move(runningAgents)},
    obstacles{getObstaclesFromJson(obstaclesJson)},
    obstaclesTimeProb{getProbabilitiesFromJson(obstaclesJson)},
    ambientMap{std::move(ambientMap)},
    agentsHistory{this->runningAgents.size()}
{}

size_t Simulator::computeSeed() const {
    size_t seed = 0;
    std::ranges::for_each(runningAgents, [&seed](const RunningAgent& ra){boost::hash_combine(seed, hash_value(ra));});
    return seed;
}

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

bool Simulator::wait(const std::vector<ObstaclePersistence>& obstaclesWithPermanence, TimeStep t, const std::unordered_set<int>& waitingAgents) {
    FixedPaths fixedPaths = extendsAndExtractFixedPaths(waitingAgents);

    auto pbsInstance{
        generatePBSInstance(obstaclesWithPermanence, t, fixedPaths, waitingAgents)
    };
    return solveWithPBS(pbsInstance, waitingAgents);
}

FixedPaths Simulator::extendsAndExtractFixedPaths(const std::unordered_set<int> &waitingAgents) {
    FixedPaths fixedPaths{};

    auto extendPath = [this](int agentId){
        assert(agentId == runningAgents[agentId].getAgentId());
        const auto& oldPath = runningAgents[agentId].getPlannedPath();

        FixedPath newPath{};
        newPath.reserve(oldPath.size() + 1);

        newPath.push_back(oldPath.front());
        newPath.insert(newPath.end(), oldPath.begin(), oldPath.end());

        // saving (side effect)
        runningAgents[agentId].setPlannedPath(newPath);
        return newPath;
    };

    std::ranges::transform(
        waitingAgents,
        std::back_inserter(fixedPaths),
        extendPath
    );
    return fixedPaths;
}

std::unordered_set<int> Simulator::getInvolvedAgents(const std::unordered_set<CompressedCoord>& actualObstacles) const {
    auto result = runningAgents |
        std::views::filter(
            [&actualObstacles](const RunningAgent& ra){
                auto nextPos = ra.getNextPosition();
                return nextPos.has_value() && actualObstacles.contains(*nextPos);
            }
        ) |
        std::views::transform([](const RunningAgent& ra){return ra.getAgentId();});

    return {result.begin(), result.end()};
}
