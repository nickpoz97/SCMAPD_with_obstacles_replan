#include <limits>
#include <numeric>
#include <cassert>
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

TimeStep Assignment::getTtd() const {
    return ttd;
}

unsigned Assignment::getIndex() const {
    return index;
}

Coord Assignment::getStartPosition() const {
    return startPosition;
}

void Assignment::setTasks(WaypointsList &&newWaypoints, const std::vector<Task> &tasks, const cmapd::AmbientMapInstance &ambientMapInstance) {
    waypoints = std::move(newWaypoints);
    internalUpdate(ambientMapInstance, tasks);
}

const WaypointsList &Assignment::getWaypoints() const {
    return waypoints;
}

bool Assignment::empty() const {
    return waypoints.empty();
}

void Assignment::setTasks(WaypointsList &&newWaypoints, const std::vector<Task> &tasks, const cmapd::AmbientMapInstance &ambientMapInstance) {
    waypoints = newWaypoints;
    internalUpdate(ambientMapInstance, tasks);
}

void
Assignment::insert(const Task &task, const cmapd::AmbientMapInstance &ambientMapInstance,
                   const std::vector<Task> &tasks,
                   Heuristic heuristic) {
    WaypointsList::iterator bestStartIt, bestGoalIt;
    TimeStep bestApproxTTD = std::numeric_limits<TimeStep>::max();

    // search for best position for task start and goal
    for(auto waypointStart = waypoints.begin(); waypointStart != waypoints.end() ; ++waypointStart){
        for (auto waypointGoal = std::next(waypointStart); waypointGoal != waypoints.end(); ++waypointGoal){
            insertTaskWaypoints(task, waypointStart, waypointGoal);
            if(checkCapacityConstraint()){
                // todo add heuristic choices
                auto newApproxTtd = computeApproxTTD(tasks, ambientMapInstance.h_table(), waypointStart, waypointGoal);
                if(newApproxTtd < bestApproxTTD){
                    bestApproxTTD = newApproxTtd;
                    bestStartIt = waypointStart;
                    bestGoalIt = waypointGoal;
                }
            }
            restorePreviousWaypoints(waypointStart, waypointGoal);
        }
    }
    auto oldTTD = ttd;

    waypoints.insert(bestStartIt, {task.startLoc, Demand::START, task.index});
    waypoints.insert(bestGoalIt, {task.goalLoc, Demand::GOAL, task.index});

    mca = ttd - oldTTD;
    internalUpdate(ambientMapInstance, tasks);

    updateHeapTop
}

void Assignment::restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                                 WaypointsList::iterator &waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

void Assignment::insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                                            WaypointsList::iterator &waypointGoal) {
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
                                    WaypointsList::const_iterator lastWaypoint, int firstIndexPath) const{
    assert(lastWaypoint != waypoints.end());

    TimeStep cumulatedTTD = 0;
    auto wpIt = waypoints.cbegin();

    // i is the timestep, if lastWaypoint is reached, break
    for(int i = firstIndexPath ; i < path.size() && wpIt != lastWaypoint; ++i){
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

    auto beforeStartIt = std::prev(startWaypoint);
    auto afterGoalIt = std::next(goalWaypoint);

    auto ttdBefore = computeRealTTD(tasks, distanceMatrix, beforeStartIt);

    auto ttdApprox = distanceMatrix.getDistance(beforeStartIt->position, startWaypoint->position) +
            distanceMatrix.getDistance(startWaypoint->position, goalWaypoint->position) +
            distanceMatrix.getDistance(goalWaypoint->position, afterGoalIt->position);

    auto ttdAfter = computeRealTTD(
            tasks, distanceMatrix,
            waypoints.cend(),
            static_cast<int>(findWaypointTimestep(path, *afterGoalIt).value())
    );

    return ttdBefore + ttdApprox + ttdAfter;

}

bool operator<(const Assignment& a, const Assignment& b){
    return a.mca < b.mca;
}

TimeStep Assignment::computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const {
    return computeRealTTD(tasks, distanceMatrix, waypoints.cend(), 0);
}

const Path &Assignment::getPath() const {
    return path;
}

std::pair<Path, std::vector<cmapd::Constraint>> Assignment::computePath(
        const cmapd::AmbientMapInstance& ambientMapInstance,
        std::vector<cmapd::Constraint>&& constraintsVector
        ) const {
    return cmapd::pbs::pbs(ambientMapInstance, std::move(constraintsVector), static_cast<int>(index), waypoints);
}

void
Assignment::internalUpdate(const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks) {
    std::tie(path, constraints) = computePath(ambientMapInstance, std::move(constraints));
    // reset ttd
    ttd = computeRealTTD(tasks, ambientMapInstance.h_table());
}

std::optional<TimeStep> Assignment::findWaypointTimestep(const Path &path, const Waypoint &waypoint, int firstIndex) {
    for(int i = firstIndex ; i < path.size() ; ++i){
        if(path[i] == waypoint.position){
            return i;
        }
    }
    return {};
}

