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

void Assignment::setTasks(WaypointsList &&newWaypoints, const std::vector<cmapd::Constraint> &outerConstraints,
                          const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks) {
    waypoints = std::move(newWaypoints);
    internalUpdate(outerConstraints, tasks, ambientMapInstance);
}

const WaypointsList &Assignment::getWaypoints() const {
    return waypoints;
}

bool Assignment::empty() const {
    return waypoints.empty();
}

void
Assignment::insert(int taskId, const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks,
                   const std::vector<cmapd::Constraint> &outerConstraints) {

    auto [bestStartIt, bestGoalIt] = findBestPositions(taskId, ambientMapInstance.h_table(), tasks);

#ifndef NDEBUG
    auto oldWaypointSize = waypoints.size();
#endif

    const auto& task = tasks[taskId];
    waypoints.insert(bestStartIt, {task.startLoc, Demand::START, task.index});
    waypoints.insert(bestGoalIt, {task.goalLoc, Demand::GOAL, task.index});
    internalUpdate(outerConstraints, tasks, ambientMapInstance);

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
                                    WaypointsList::const_iterator firstWaypoint,
                                    WaypointsList::const_iterator lastWaypoint) const{
    TimeStep cumulatedTTD = 0;
    auto wpIt = firstWaypoint;

    // i is the timestep, if lastWaypoint is reached, break
    for(int i = 0 ; i < path.size() && wpIt != lastWaypoint; ++i){
        // reached waypoint
        if(path[i] == wpIt->position){
            if(wpIt->demand == Demand::GOAL){
                const Task& task = tasks[wpIt->taskIndex];
                cumulatedTTD += i - task.getIdealGoalTime(distanceMatrix);
            }
            ++wpIt;
        }
    }

    return cumulatedTTD;
}

TimeStep Assignment::computeApproxTTD(
    const std::vector<Task> &tasks,
    const DistanceMatrix &distanceMatrix,
    WaypointsList::const_iterator startWaypoint,
    WaypointsList::const_iterator goalWaypoint
    ) const{

    TimeStep ttd = 0;

    assert(startWaypoint != waypoints.cend() && goalWaypoint != waypoints.cbegin());

    std::array<WaypointsList::const_iterator, 4> checkpoints{};

    checkpoints[0] = startWaypoint != waypoints.cbegin() ? std::prev(startWaypoint) : startWaypoint;
    checkpoints[1] = std::next(startWaypoint);
    checkpoints[2] = goalWaypoint != waypoints.cend() ? std::next(goalWaypoint) : goalWaypoint;
    checkpoints[3] = checkpoints[2] != waypoints.cend() ? std::next(checkpoints[2]) : checkpoints[2];

    ttd += computeRealTTD(tasks, distanceMatrix, waypoints.cbegin(), checkpoints[0]);
    ttd += distanceMatrix.getDistance(checkpoints[0]->position, startWaypoint->position);
    ttd += distanceMatrix.getDistance(startWaypoint->position, checkpoints[1]->position);
    ttd += computeRealTTD(tasks, distanceMatrix, checkpoints[1], checkpoints[2]);
    ttd += distanceMatrix.getDistance(checkpoints[2]->position, goalWaypoint->position);
    ttd += distanceMatrix.getDistance(goalWaypoint->position, checkpoints[3]->position);
    ttd += computeRealTTD(tasks, distanceMatrix, checkpoints[3], waypoints.end());

    return ttd;
}

bool operator<(const Assignment& a, const Assignment& b){
    return a.getMCA() < b.getMCA();
}

TimeStep Assignment::computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const {
    return computeRealTTD(tasks, distanceMatrix, waypoints.cbegin(), waypoints.cend());
}

const Path &Assignment::getPath() const {
    return path;
}

std::pair<Path, std::vector<cmapd::Constraint>> Assignment::computePath(
        const cmapd::AmbientMapInstance& ambientMapInstance,
        const std::vector<cmapd::Constraint> &outerConstraints
        ) const {
    return cmapd::pbs::pbs(ambientMapInstance, outerConstraints, static_cast<int>(index), waypoints);
}

void
Assignment::internalUpdate(const std::vector<cmapd::Constraint> &outerConstraints, const std::vector<Task> &tasks,
                           const cmapd::AmbientMapInstance &ambientMapInstance) {
    oldTTD = newTTD;

    std::tie(path, constraints) = computePath(ambientMapInstance, outerConstraints);
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

const std::vector<cmapd::Constraint> &Assignment::getConstraints() const {
    return constraints;
}

bool Assignment::hasConflicts(const Assignment& a, const Assignment& b){
    const auto& pathA = a.getPath();
    const auto& pathB = b.getPath();

    // same robot
    if(a.getIndex() == b.getIndex()){
        return false;
    }

    for (int i = 0 ; i < std::min(pathA.size(), pathB.size()); ++i){
        if(checkConflict(pathA, pathB, i)) { return true; }
    }
    return false;
}

bool Assignment::checkConflict(const Path& a, const Path& b, int i) {
    bool edgeConflict = (i > 0) && a[i - 1] == b[i] && a[i] == b[i - 1];
    bool nodeConflict = a[i] == b[i];

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
