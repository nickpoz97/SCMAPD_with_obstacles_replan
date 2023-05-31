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
                default:
                    throw std::runtime_error("Not handled case");
                break;
            }
        }

        std::ranges::for_each(runningAgents, [](RunningAgent& ra){ra.stepAndUpdate();});
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

Instance Simulator::generatePBSInstance(const std::vector<ObstaclePersistence> &obstaclesWithPermanence,
                                        TimeStep actualTimeStep) const {
    auto checkPointsExtractor = [](const RunningAgent& ra) -> Path {
        std::vector<CompressedCoord> checkPoints{ra.getActualPosition()};

        std::ranges::copy(
            ra.getPlannedCheckpoints(),
            std::back_inserter(checkPoints)
        );

        return checkPoints;
    };

    std::vector<Path> agentsCheckpoints{};

    std::ranges::transform(runningAgents, std::back_inserter(agentsCheckpoints), checkPointsExtractor);
    assert(agentsCheckpoints.size() == runningAgents.size());

    // extract grid
    auto grid{ambientMap.getGrid()};
    assert(grid.size() == ambientMap.getNRows() * ambientMap.getNCols());

    return {
        grid,
        agentsCheckpoints,
        ambientMap.getNRows(),
        ambientMap.getNCols(),
        getSpawnedObstacles(obstaclesWithPermanence, actualTimeStep)
    };
}

boost::unordered_set<SpawnedObstacle>
Simulator::getSpawnedObstacles(const vector<ObstaclePersistence> &obstaclesWithPermanence,
                               TimeStep actualTimeStep) {
    boost::unordered::unordered_set<SpawnedObstacle> spawnedObstacles{};
    for (const auto& owp : obstaclesWithPermanence){
        for(int t = actualTimeStep ; t < actualTimeStep + owp.duration ; ++t){
            spawnedObstacles.emplace(t, owp.loc);
        }
    }
    return spawnedObstacles;
}

void Simulator::updatePlannedPaths(const std::vector<Path>& paths) {
    for (int i = 0 ; i < paths.size() ; i++){
        assert(i = runningAgents[i].getAgentId());
        runningAgents[i].setPlannedPath(paths[i]);
    }
}

bool Simulator::rePlan(const std::vector<ObstaclePersistence>& obstaclesWithPermanence, TimeStep t) {
    PBS pbs{generatePBSInstance(obstaclesWithPermanence, t), true, 0};
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
    ambientMap{std::move(ambientMap)}
{
}

size_t Simulator::computeSeed() const {
    size_t seed = 0;
    std::ranges::for_each(runningAgents, [&seed](const RunningAgent& ra){boost::hash_combine(seed, hash_value(ra));});
    return seed;
}
