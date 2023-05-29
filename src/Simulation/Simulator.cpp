//
// Created by nicco on 27/05/2023.
//

#include <boost/tokenizer.hpp>
#include "Simulation/Simulator.hpp"
#include "PBS.h"

void Simulator::simulate(size_t hash, Strategy strategy) {
    using std::ranges::any_of;

    while(any_of(runningAgents, [](const RunningAgent& ra){return !ra.hasFinished();})){
        // extract obstacles at this timeStep
        auto actualObstacles = obstacles.front();
        obstacles.pop_front();

        // obstacles in that our agent will cross in this iteration
        std::vector<CompressedCoord> foundObstacles{};
        std::ranges::set_intersection(getNextPositions(), actualObstacles, std::back_inserter(foundObstacles));

        if(!foundObstacles.empty()){
            switch (strategy) {
                // warning cannot solve if obstacles is on a target
                case Strategy::RE_PLAN:
                    rePlan(actualObstacles);
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

Instance Simulator::generatePBSInstance(const std::vector<CompressedCoord>& obstaclesLocation) const {
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

    // each position with an obstacle is considered an obstacle
    std::ranges::for_each(obstaclesLocation, [&grid](CompressedCoord obstacle){grid[obstacle] = true;});

    return {grid, agentsCheckpoints, ambientMap.getNRows(), ambientMap.getNCols()};
}

void Simulator::updatePlannedPaths(const std::vector<Path>& paths) {
    for (int i = 0 ; i < paths.size() ; i++){
        assert(i = runningAgents[i].getAgentId());
        runningAgents[i].setPlannedPath(paths[i]);
    }
}

bool Simulator::rePlan(const std::vector<CompressedCoord>& actualObstacles) {
    PBS pbs{generatePBSInstance(actualObstacles), true, 0};
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
