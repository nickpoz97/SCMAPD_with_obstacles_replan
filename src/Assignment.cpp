#include <limits>
#include <array>
#include <numeric>
#include <cassert>
#include <algorithm>
#include <fstream>
#include <fmt/core.h>
#include <fmt/ranges.h>

#include "Assignment.hpp"
#include "pbs.h"

Assignment::Assignment(Coord startPosition, int index, int capacity) :
        startPosition{startPosition},
        index{index},
        capacity{capacity}
    {}

int Assignment::getCapacity() const {
    return capacity;
}

TimeStep Assignment::getMCA() const {
    return newTTD - oldTTD;
}

int Assignment::getIndex() const {
    return index;
}

Coord Assignment::getStartPosition() const {
    return startPosition;
}

bool Assignment::empty() const {
    return waypoints.empty();
}

void
Assignment::addTask(const cmapd::AmbientMapInstance &ambientMapInstance,
                    const std::vector<std::vector<cmapd::Constraint>> &constraints, const Task &task) {

    auto [bestStartIt, bestGoalIt] = findBestPositions(task, ambientMapInstance.h_table());

#ifndef NDEBUG
    auto oldWaypointSize = waypoints.size();
#endif

    waypoints.insert(bestStartIt, {task.startLoc, Demand::START, task.index});
    waypoints.insert(bestGoalIt, {task.goalLoc, Demand::GOAL, task.index});
    internalUpdate(constraints, tasks, ambientMapInstance, true);

#ifndef NDEBUG
    assert(oldWaypointSize == waypoints.size() - 2);
    int sum = 0;
    for(const auto& w : waypoints){
        sum += static_cast<int>(w.demand);
    }
    assert(sum == 0);
#endif
}

std::pair<WaypointsList::iterator, WaypointsList::iterator>
Assignment::findBestPositions(const Task &task, const DistanceMatrix &distanceMatrix) {
    TimeStep bestApproxTTD = std::numeric_limits<decltype(bestApproxTTD)>::max();

    // we must use end iterator position to explore all possible combinations
    auto nIterations = waypoints.size() + 1;
    int i = 0;
    int j = 0;
    auto waypointStart = waypoints.begin();
    auto waypointGoal = waypointStart;

    auto bestStartIt = waypointStart;
    auto bestGoalIt = waypointGoal;

    // search for best position for task start and goal
    for(; i < nIterations ; ++waypointStart, ++i){
        for (; j < nIterations; ++waypointGoal, ++j){
            auto [newStartIt, newGoalIt] = insertNewWaypoints(task, waypointStart, waypointGoal);
            if(checkCapacityConstraint()){
                auto newApproxTtd = computeApproxTTD(task, distanceMatrix, newStartIt, newGoalIt);
                if(newApproxTtd < bestApproxTTD){
                    bestApproxTTD = newApproxTtd;
                    bestStartIt = waypointStart;
                    bestGoalIt = waypointGoal;
                }
            }
            restorePreviousWaypoints(newStartIt, newGoalIt);
        }
    }
    return {bestStartIt, bestGoalIt};
}

void Assignment::restorePreviousWaypoints(std::_List_iterator<Waypoint> waypointStart,
                                          std::_List_iterator<Waypoint> waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

std::pair<WaypointsList::iterator, WaypointsList::iterator> Assignment::insertNewWaypoints(const Task &task, std::_List_iterator<Waypoint> waypointStart,
                                                                                           std::_List_iterator<Waypoint> waypointGoal) {
    return {
        waypoints.insert(waypointStart, {task.startLoc, Demand::START, task.index}),
        waypoints.insert(waypointGoal, {task.goalLoc, Demand::GOAL, task.index})
    };
}

bool Assignment::checkCapacityConstraint() {
    int actualWeight = 0;

    for(const auto& waypoint : waypoints){
        actualWeight += static_cast<int>(waypoint.demand);
        if(actualWeight > getCapacity()){
            return false;
        }
    }
    return true;
}

TimeStep Assignment::computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix,
                                    const std::_List_iterator<Waypoint> &firstWaypoint,
                                    const std::_List_iterator<Waypoint> &lastWaypoint){
    TimeStep cumulatedTTD = 0;
    auto wpIt = firstWaypoint;

    // i is the timestep, if lastWaypoint is reached, break
    for(int i = 0 ; i < path.size() && wpIt != lastWaypoint; ++i){
        // reached waypoint
        if(path[i] == wpIt->position){
            if(wpIt->demand == Demand::GOAL){
                const Task& task = tasks[wpIt->taskIndex];
                auto delay = i - task.getIdealGoalTime();
                cumulatedTTD += delay;
                wpIt->setDelay(delay);
            }
            ++wpIt;
        }
    }

    return cumulatedTTD;
}

TimeStep Assignment::computeApproxTTD(
        const Task &task,
        const DistanceMatrix &distanceMatrix,
        WaypointsList::iterator startWaypoint,
        WaypointsList::iterator goalWaypoint
    ) const{

    TimeStep ttd = 0;

    assert(startWaypoint != waypoints.cend() && goalWaypoint != waypoints.cbegin() && goalWaypoint != waypoints.cend());

    auto prevWpIt = [this](WaypointsList::const_iterator it){
        return it == waypoints.cbegin() ? it : std::prev(it);
    };
    auto nextWpIt = [this](WaypointsList::const_iterator it){
        return std::next(it) == waypoints.cend() ? it : std::next(it);
    };

    auto wpIt = waypoints.cbegin();
    int iApprox = 0;

    for(int i = 0 ; i < path.size() && wpIt != waypoints.cend() ; ++i){
        if(path[i] == wpIt->position) {
            if (wpIt == startWaypoint) {
                iApprox += distanceMatrix.getDistance(prevWpIt(startWaypoint)->position, startWaypoint->position);
                auto afterStartIt = nextWpIt(startWaypoint);
                if(afterStartIt != goalWaypoint) {
                    iApprox += distanceMatrix.getDistance(startWaypoint->position, afterStartIt->position);
                }
            }
            if (wpIt == goalWaypoint) {
                iApprox += distanceMatrix.getDistance(prevWpIt(goalWaypoint)->position, goalWaypoint->position);
                // todo check this
                ttd += (i + iApprox) - task.getIdealGoalTime();
                iApprox += distanceMatrix.getDistance(goalWaypoint->position, nextWpIt(goalWaypoint)->position);
                ++wpIt;
            }
            if (wpIt->demand == Demand::GOAL) {
                ttd += (i + iApprox) - task.getIdealGoalTime();
            }
            ++wpIt;
        }
    }

    return ttd;
}

bool operator<(const Assignment& a, const Assignment& b){
    return a.getMCA() < b.getMCA();
}

TimeStep Assignment::computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) {
    return computeRealTTD(tasks, distanceMatrix, waypoints.begin(), waypoints.end());
}

const Path &Assignment::getPath() const {
    return path;
}

void
Assignment::internalUpdate(const std::vector<std::vector<cmapd::Constraint>> &constraints, const std::vector<Task> &tasks,
                           const cmapd::AmbientMapInstance &ambientMapInstance, bool newTasks) {
    if(newTasks) { oldTTD = newTTD; }

    path = cmapd::pbs::pbs(ambientMapInstance, constraints, index, waypoints);
    // reset ttd
    newTTD = computeRealTTD(tasks, ambientMapInstance.h_table());
}

std::vector<cmapd::Constraint>
Assignment::getConstraints(const cmapd::AmbientMapInstance &instance) const {
    std::vector<cmapd::Constraint> constraints;

    for (int timestep = 0 ; timestep < path.size(); ++timestep) {
        const auto& point = path[timestep];
        for (const auto& move : moves) {
            auto from_where{point + move};
            if (instance.is_valid(from_where)) {
                // if this is the last timestep, final should equal to true
                constraints.push_back({timestep, from_where, point, timestep == path.size() - 1});
                // todo add edge constraint
            }
        }
    }

    return constraints;
}

Assignment::operator std::string() const{
    auto [firstDiv, lastDiv] = utils::buildDivider("Assignment");

    return fmt::format(
        "{}\n{}\n{}\n{}\n{}\n{}\n{}\n",
        firstDiv,
        fmt::format("Index: {}", index),
        fmt::format("Waypoints: {}", utils::objContainerString(waypoints)),
        fmt::format("Path: {}", utils::objContainerString(path)),
        fmt::format("Path Length: {}", path.size()),
        fmt::format("newTTD - oldTTD = {} - {} = {}", newTTD, oldTTD, getMCA()),
        lastDiv
    );
}

bool Assignment::hasConflicts(const std::vector<cmapd::Constraint> &constraints) const {
    for(int i = 0 ; i < path.size() ; ++i){
        for(const auto& c : constraints){
            conflictsWith(path, i, c);
        }
    }

    return false;
}

bool Assignment::conflictsWith(const Path &path, TimeStep i, const cmapd::Constraint &c) {
    auto nextI = std::min(static_cast<int>(path.size()) - 1, i);
    return i < path.size() && path[i] == c.from_position && path[nextI] == c.to_position;
}

std::vector<Assignment>
loadAssignments(const std::filesystem::path &agentsFilePath, char horizontalSep, int capacity) {
    std::ifstream fs (agentsFilePath, std::ios::in);
    std::string line;

    // nAgents line
    std::getline(fs, line);
    size_t nAgents = std::stoi(line);

    std::vector<Assignment> agents;
    agents.reserve(nAgents);

    for (int i = 0 ; i < nAgents; ++i){
        std::getline(fs, line);

        std::string xCoordString, yCoordString;
        auto coordStream = std::stringstream(line);
        std::getline(coordStream, yCoordString, horizontalSep);
        std::getline(coordStream, xCoordString, horizontalSep);

        //CompressedCoord cc = DistanceMatrix::from2Dto1D(std::stoi(xCoordString), std::stoi(yCoordString), nCols);

        Coord position{std::stoi(yCoordString), std::stoi(xCoordString)};
        agents.emplace_back(position, i, capacity);
    }

    return agents;
}
