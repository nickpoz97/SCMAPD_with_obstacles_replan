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

        auto nextPositions = getNextPositions();

        // extract obstacles at this timeStep (empty forward list otherwise)
        const auto& actualObstacles = obstaclesWrapper.updateAndGet(t, nextPositions, strategy != Strategy::WAIT);

        if(!actualObstacles.empty() || rePlanAfterWait){
            const auto involvedAgents{getInvolvedAgents(actualObstacles)};

            switch (strategy) {
                // warning cannot solve if obstacles is on a target
                case Strategy::RE_PLAN:
                    rePlan(actualObstacles);
                break;
                case Strategy::WAIT:
                    if(!actualObstacles.empty()){
                        wait(
                            actualObstacles,
                            involvedAgents
                        );
                    }
                    else{
                        rePlan(actualObstacles);
                    }
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
        std::views::transform( [](const auto& ra){return ra.getNextPosition();} );

    return {nextPositions.begin(), nextPositions.end()};
}

Instance
Simulator::generatePBSInstance(const SpawnedObstaclesSet &sOSet, const std::unordered_set<int> &waitingAgents) const {
    vector<Path> agentsCheckpoints = extractPBSCheckpoints(waitingAgents);

    auto grid{ambientMap.getGrid()};

    for (CompressedCoord aId : waitingAgents){
        assert(agentsCheckpoints[aId].size() == 1);
        grid[agentsCheckpoints[aId].front()] = true;
    }

    return {
        std::move(grid),
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
        int agentId = actualAgent.getAgentId();
        actualAgent.setPlannedPath(paths[agentId]);
        //assert(actualAgent.checkpointChecker());
    }
}

void Simulator::rePlan(const SpawnedObstaclesSet &sOSet) {
    rePlanAfterWait = false;

    auto pbsInstance{generatePBSInstance(sOSet, {})};
    updatePlannedPaths(solveWithPBS(pbsInstance));
}

std::vector<Path> Simulator::solveWithPBS(const Instance &pbsInstance) {
    PBS pbs{pbsInstance, true, 0};
    if(pbs.solve(7200)){
        return pbs.getPaths();
    }
    throw std::runtime_error("No Path");
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

void Simulator::wait(const SpawnedObstaclesSet &spawnedObstacles, const std::unordered_set<int> &waitingAgents) {
    rePlanAfterWait = true;

    std::unordered_map<int, CompressedCoord> formerNextPos;
    std::ranges::for_each(
        waitingAgents,
        [&formerNextPos, this](int aId){formerNextPos[aId] = runningAgents[aId].getNextPosition();}
    );

    auto pbsInstance{
        generatePBSInstance(spawnedObstacles, waitingAgents)
    };
    updatePlannedPaths(solveWithPBS(pbsInstance));

    // extend with waiting pos
    std::ranges::for_each(
        formerNextPos,
        [this](const auto& kv) {
            auto [aId, nextPos] = kv;
            auto& actualAgent = runningAgents[aId];
            auto extendedPath = actualAgent.getPlannedPath();
            assert(extendedPath.size() == 1);

            extendedPath.push_back(extendedPath.front());
            extendedPath.push_back(nextPos);

            actualAgent.setPlannedPath(extendedPath);
        }
    );
}

std::unordered_set<int>
Simulator::getInvolvedAgents(const SpawnedObstaclesSet &actualObstacles) const {
    auto result = runningAgents |
        std::views::filter(
            [&actualObstacles](const RunningAgent& ra){
                auto nextPos = ra.getNextPosition();
                return actualObstacles.contains({1, nextPos});
            }
        ) |
        std::views::transform([](const RunningAgent& ra){return ra.getAgentId();});

    return {result.begin(), result.end()};
}

Interval Simulator::getScore(const std::vector<CompressedCoord> &obstaclesPositions, bool useMakespan) const {
    double score = 0;
    for(const auto pos : obstaclesPositions){

        for(const auto [permanence, p] : obstaclesWrapper.getProbabilities(pos)){
            SpawnedObstaclesSet sOSet;
            for(Interval i = 0 ; i < permanence ; ++i){
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
    return static_cast<Interval>(score) - getResultPenalty(useMakespan, idealPaths);
}

size_t Simulator::getResultPenalty(bool useMakespan, const vector<Path> &paths) {
    size_t value = useMakespan ?
         std::ranges::max_element(paths, [](const Path& a, const Path& b){return a.size() < b.size();})->size() :
         std::accumulate(paths.cbegin(), paths.cend(), 0, [](size_t sum, const Path& path){return sum + path.size();});
    return value;
}
