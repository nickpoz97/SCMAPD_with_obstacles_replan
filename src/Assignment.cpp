#include <limits>
#include <array>
#include <numeric>
#include <cassert>
#include <algorithm>
#include <fstream>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <set>

#include "Assignment.hpp"
#include "MAPF/MultiAStar.hpp"

Assignment::Assignment(const AgentInfo &agentInfo, int firstTaskId, const Status &status) :
        startPos{agentInfo.startPos},
        waypoints{},
        index{agentInfo.index},
        capacity{agentInfo.capacity}
    {
        addTask(firstTaskId, status);
        assert(path.size() > 2 && path[0] == startPos && waypoints.size() == 2);
        assert(agentInfo.index == index);
    }

int Assignment::getCapacity() const {
    return capacity;
}

TimeStep Assignment::getMCA() const {
    return getActualTTD() - oldTTD;
}

int Assignment::getAgentId() const {
    return index;
}

CompressedCoord Assignment::getStartPosition() const {
    return startPos;
}

bool Assignment::empty() const {
    return waypoints.empty();
}

void
Assignment::addTask(int taskId, const Status &status) {

#ifndef NDEBUG
    auto oldWaypointSize = waypoints.size();
#endif
    oldTTD = getActualTTD();
    insertTaskWaypoints(taskId, status);

    MultiAStar pathfinder{};
    std::tie(path, waypoints) = pathfinder.solve(std::move(waypoints), startPos, status, index);

#ifndef NDEBUG
    assert(oldWaypointSize == waypoints.size() - 2);
    int sum = 0;
    for(const auto& w : waypoints){
        sum += static_cast<int>(w.demand);
    }
    assert(sum == 0);
#endif
}

void
Assignment::insertTaskWaypoints(int taskId, const Status &status) {
    const Task& task = status.getTask(taskId);

    if(waypoints.empty()){
        waypoints = getTaskWaypoints(task);
        return;
    }

    // we must use end iterator position to explore all possible combinations
    auto lastIteration = waypoints.size() + 1;

    auto wpPickupIt = waypoints.begin();
    auto wpDeliveryIt = wpPickupIt;

    auto bestPickupIt = wpPickupIt;
    auto bestDeliveryIt = wpDeliveryIt;

    TimeStep bestApproxTTD = std::numeric_limits<decltype(bestApproxTTD)>::max();

    // todo fix this waypoints continuously grow
    // search for best position for task start and goal
    for(int i = 0; i < lastIteration ; ++wpPickupIt, ++i){
        for (int j = i+1; j < lastIteration; ++wpDeliveryIt, ++j){
            auto [newStartIt, newGoalIt] = insertNewWaypoints(task, wpPickupIt, wpDeliveryIt);
            if(checkCapacityConstraint()){
                auto newApproxTtd = computeApproxTTD(status, newStartIt);
                if(newApproxTtd < bestApproxTTD){
                    bestApproxTTD = newApproxTtd;
                    bestPickupIt = wpPickupIt;
                    bestDeliveryIt = wpDeliveryIt;
                }
            }
            restorePreviousWaypoints(newStartIt, newGoalIt);
        }
    }
    insertNewWaypoints(task, bestPickupIt, bestDeliveryIt);
}

void Assignment::restorePreviousWaypoints(std::_List_iterator<Waypoint> waypointStart,
                                          std::_List_iterator<Waypoint> waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

std::pair<WaypointsList::iterator, WaypointsList::iterator> Assignment::insertNewWaypoints(const Task &task, std::_List_iterator<Waypoint> waypointStart,
                                                                                           std::_List_iterator<Waypoint> waypointGoal) {
    return {
        waypoints.insert(waypointStart, {task.startLoc, Demand::PICKUP, task.index}),
        waypoints.insert(waypointGoal, {task.goalLoc, Demand::DELIVERY, task.index})
    };
}

bool Assignment::checkCapacityConstraint() {
    int actualWeight = 0;

    for(const auto& waypoint : waypoints){
        actualWeight += static_cast<int>(waypoint.demand);
        if(actualWeight > capacity || actualWeight < 0){
            return false;
        }
    }
    return true;
}

TimeStep Assignment::getActualTTD() const{
    return waypoints.empty() ? 0 :waypoints.crbegin()->getCumulatedDelay();
}

TimeStep Assignment::computeApproxTTD(const Status &status, WaypointsList::iterator newPickupWpIt) const{

    assert(newPickupWpIt != waypoints.end());

    const auto& dm = status.getDistanceMatrix();

    auto ttd = newPickupWpIt == waypoints.begin() ? 0 : std::prev(newPickupWpIt)->getCumulatedDelay();
    auto prevWpPos = newPickupWpIt == waypoints.begin() ? startPos : std::prev(newPickupWpIt)->position;
    auto prevArrivalTime = newPickupWpIt == waypoints.begin() ? 0 : std::prev(newPickupWpIt)->getArrivalTime();

    for(auto wpIt = newPickupWpIt ; wpIt != waypoints.end() ; ++wpIt){
        if(wpIt->demand == Demand::DELIVERY){
            // using ideal path
            // todo fix this (ttd is negative)
            auto arrivalTime = prevArrivalTime + dm.getDistance(prevWpPos, wpIt->position);
            ttd += arrivalTime - status.getTask(wpIt->taskIndex).idealGoalTime;
            prevWpPos = wpIt->position;
            prevArrivalTime = arrivalTime;
        }
    }

    return ttd;
}

bool operator<(const Assignment& a, const Assignment& b){
    return a.getMCA() < b.getMCA();
}

PathWrapper Assignment::extractAndReset() {
    waypoints.clear();
    oldTTD = 0;
    return {index, std::exchange(path, {})};
}

void
Assignment::internalUpdate(const Status &status) {
    MultiAStar pathfinder{};
    std::tie(path, waypoints) = pathfinder.solve(std::move(waypoints), startPos, status, index);
}

const WaypointsList &Assignment::getWaypoints() const {
    return waypoints;
}

const Path &Assignment::getPath() const {
    return path;
}

bool operator>(const Assignment &a, const Assignment &b) {
    return a.getMCA() > b.getMCA();
}
