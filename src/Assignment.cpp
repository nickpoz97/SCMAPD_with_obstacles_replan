#include <limits>
#include <array>
#include <numeric>
#include <cassert>
#include <algorithm>

#include "Assignment.hpp"
#include "pbs.h"

Assignment::Assignment(Coord startPosition, unsigned index, unsigned capacity) :
        startPosition{startPosition},
        index{index},
        capacity{capacity}
    {}

unsigned int Assignment::getCapacity() const {
    return capacity;
}

TimeStep Assignment::getMCA() const {
    return newTTD - oldTTD;
}

unsigned Assignment::getIndex() const {
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

    const auto& task = tasks[taskId];
    waypoints.insert(bestStartIt, {task.startLoc, Demand::START, task.index});
    waypoints.insert(bestGoalIt, {task.goalLoc, Demand::GOAL, task.index});
    internalUpdate(outerConstraints, tasks, ambientMapInstance);
}

std::pair<WaypointsList::iterator, WaypointsList::iterator>
Assignment::findBestPositions(int taskId, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks) {
    WaypointsList::iterator bestStartIt;
    WaypointsList::iterator bestGoalIt;

    TimeStep bestApproxTTD = std::numeric_limits<decltype(bestApproxTTD)>::max();

    // search for best position for task start and goal
    for(auto waypointStart = waypoints.begin(); waypointStart != waypoints.end() ; ++waypointStart){
        for (auto waypointGoal = std::next(waypointStart); waypointGoal != waypoints.end(); ++waypointGoal){
            insertTaskWaypoints(tasks[taskId], waypointStart, waypointGoal);
            if(checkCapacityConstraint()){
                auto newApproxTtd = computeApproxTTD(tasks, distanceMatrix, waypointStart, waypointGoal);
                if(newApproxTtd < bestApproxTTD){
                    bestApproxTTD = newApproxTtd;
                    bestStartIt = waypointStart;
                    bestGoalIt = waypointGoal;
                }
            }
            restorePreviousWaypoints(waypointStart, waypointGoal);
        }
    }
    return {bestStartIt, bestGoalIt};
}

void Assignment::restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                                 WaypointsList::iterator &waypointGoal) {
    waypointStart = waypoints.erase(waypointStart);
    waypointGoal = waypoints.erase(waypointGoal);
}

void Assignment::insertTaskWaypoints(const Task &task, std::_List_iterator<Waypoint> waypointStart,
                                     std::_List_iterator<Waypoint> waypointGoal) {
    waypoints.insert(waypointStart, {task.startLoc, Demand::START, task.index});
    waypoints.insert(waypointGoal, {task.goalLoc, Demand::GOAL, task.index});
}

bool Assignment::checkCapacityConstraint() {
    unsigned actualWeight = 0;

    for(const auto& waypoint : waypoints){
        actualWeight += static_cast<unsigned>(waypoint.demand);
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
            wpIt = std::next(wpIt);
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
