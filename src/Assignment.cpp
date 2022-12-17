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

const WaypointsList &Assignment::getWaypoints() const {
    return waypoints;
}

bool Assignment::empty() const {
    return waypoints.empty();
}

void
Assignment::insert(int taskId, const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks,
                   const std::vector<std::vector<cmapd::Constraint>> &outerConstraints) {

    auto [bestStartIt, bestGoalIt] = findBestPositions(taskId, ambientMapInstance.h_table(), tasks);

#ifndef NDEBUG
    auto oldWaypointSize = waypoints.size();
#endif

    const auto& task = tasks[taskId];
    waypoints.insert(bestStartIt, {task.startLoc, Demand::START, task.index});
    waypoints.insert(bestGoalIt, {task.goalLoc, Demand::GOAL, task.index});
    internalUpdate(outerConstraints, tasks, ambientMapInstance, true);

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
Assignment::findBestPositions(int taskId, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks) {
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
            auto [newStartIt, newGoalIt] = insertNewWaypoints(tasks[taskId], waypointStart, waypointGoal);
            if(checkCapacityConstraint()){
                auto newApproxTtd = computeApproxTTD(tasks, distanceMatrix, newStartIt, newGoalIt);
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
                auto delay = i - task.getIdealGoalTime(distanceMatrix);
                cumulatedTTD += delay;
                wpIt->setDelay(delay);
            }
            ++wpIt;
        }
    }

    return cumulatedTTD;
}

TimeStep Assignment::computeApproxTTD(
        const std::vector<Task> &tasks,
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
                ttd += (i + iApprox) - tasks[wpIt->taskIndex].getIdealGoalTime(distanceMatrix);
                iApprox += distanceMatrix.getDistance(goalWaypoint->position, nextWpIt(goalWaypoint)->position);
                ++wpIt;
            }
            if (wpIt->demand == Demand::GOAL) {
                ttd += (i + iApprox) - tasks[wpIt->taskIndex].getIdealGoalTime(distanceMatrix);
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
Assignment::internalUpdate(const std::vector<std::vector<cmapd::Constraint>> &outerConstraints, const std::vector<Task> &tasks,
                           const cmapd::AmbientMapInstance &ambientMapInstance, bool newTasks) {
    if(newTasks) { oldTTD = newTTD; }

    path = cmapd::pbs::pbs(ambientMapInstance, outerConstraints, index, waypoints);
    // reset ttd
    newTTD = computeRealTTD(tasks, ambientMapInstance.h_table());
}

std::optional<TimeStep> Assignment::findWaypointTimestep(const Path &path, const Waypoint &waypoint) {
    for(int i = 0 ; i < path.size() ; ++i){
        if(path[i] == waypoint.position){
            return i;
        }
    }
    return {};
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
            }
        }
    }

    return constraints;
}

bool Assignment::hasConflicts(const Assignment& a, const Assignment& b){
    if(a.empty() || b.empty()){
        return false;
    }

    const auto& pathA = a.getPath();
    const auto& pathB = b.getPath();

    // same robot
    if(a.getIndex() == b.getIndex()){
        return false;
    }

    for (int i = 0 ; i < std::max(pathA.size(), pathB.size()); ++i){
        if(checkConflict(pathA, pathB, i)) { return true; }
    }
    return false;
}

bool Assignment::checkConflict(const Path& a, const Path& b, int i) {
    auto checkAndFix = [](int t, const Path& p){
        auto lastElement = p.size() - 1;
        return t <= lastElement ? p[t] : p[lastElement];
    };

    bool edgeConflict = (i > 0) && checkAndFix(i-1, a) == checkAndFix(i, b) && checkAndFix(i, a) == checkAndFix(i-1, b);
    bool nodeConflict = checkAndFix(i, a) == checkAndFix(i,b);

    return edgeConflict || nodeConflict;
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

bool Assignment::pathContainsErrors(const std::vector<std::vector<cmapd::Constraint>>& constraints) const {
    for(int i = 0 ; i < path.size() ; ++i){
        for(const auto& c : constraints){
            cmapd::Constraint pointC{i, path[i], path[i]};

        }
    }
}

bool conflictsWith(const Path &path, TimeStep i, const cmapd::Constraint &c) {
    return i < path.size() && path[i] == c.from_position && path[i+1] == c.to_position;
}

std::vector<Assignment>
loadAssignments(const std::filesystem::path &agentsFilePath, int nCols, char horizontalSep, int capacity){
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
