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
#include "MAPF/PathFinder.hpp"
#include "MAPF/NoPathException.hpp"

Assignment::Assignment(const AgentInfo &agentInfo, int firstTaskId, const Status &status) :
        startPos{agentInfo.startPos},
        waypoints{Waypoint{agentInfo.startPos}},
        index{agentInfo.index},
        capacity{agentInfo.capacity}
    {
        addTask(firstTaskId, status);
        assert(path.size() > 0 && path[0] == startPos && waypoints.size() == 3);
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
    idealGoalTime = computeIdealGoalTime(status);
    std::tie(path, waypoints) = PathFinder::multiAStar(std::move(waypoints), startPos, status, index);

    assert(!status.checkPathWithStatus(path, index));
#ifndef NDEBUG
    assert(oldWaypointSize == waypoints.size() - 2);
    int sum = 0;
    for(const auto& w : waypoints){
        sum += static_cast<int>(w.getDemand());
    }
    assert(sum == 0);
#endif
}

void
Assignment::insertTaskWaypoints(int taskId, const Status &status) {
    const Task& task = status.getTask(taskId);
    assert(waypoints.crend()->getDemand() == Demand::END);

    if(waypoints.size() == 1){
        waypoints.push_front(getTaskDeliveryWaypoint(task));
        waypoints.push_front(getTaskPickupWaypoint(task));
        return;
    }

    // we must use end iterator position to explore all possible combinations
    auto nIterations = waypoints.size() + 1;

    auto bestPickupIt = waypoints.begin();
    auto bestDeliveryIt = bestPickupIt;

    TimeStep bestApproxTTD = std::numeric_limits<decltype(bestApproxTTD)>::max();

    // search for best position for task start and goal
    for(auto wpPickupIt = waypoints.begin(); wpPickupIt != waypoints.end() ; ++wpPickupIt){
        for (auto wpDeliveryIt = wpPickupIt; wpDeliveryIt != waypoints.cend(); ++wpDeliveryIt){
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
        waypoints.insert(waypointStart, getTaskPickupWaypoint(task)),
        waypoints.insert(waypointGoal, getTaskDeliveryWaypoint(task))
    };
}

bool Assignment::checkCapacityConstraint() {
    int actualWeight = 0;

    for(const auto& waypoint : waypoints){
        actualWeight += static_cast<int>(waypoint.getDemand());
        if(actualWeight > capacity || actualWeight < 0){
            return false;
        }
    }
    return true;
}

TimeStep Assignment::getActualTTD() const{
    assert(!waypoints.empty());
    return waypoints.crbegin()->getCumulatedDelay();
}

TimeStep Assignment::computeApproxTTD(const Status &status, WaypointsList::iterator newPickupWpIt) const{

    assert(newPickupWpIt != waypoints.end());

    const auto& dm = status.getDistanceMatrix();

    auto ttd = newPickupWpIt == waypoints.begin() ? 0 : std::prev(newPickupWpIt)->getCumulatedDelay();
    auto prevWpPos = newPickupWpIt == waypoints.begin() ? startPos : std::prev(newPickupWpIt)->getPosition();
    auto prevArrivalTime = newPickupWpIt == waypoints.begin() ? 0 : std::prev(newPickupWpIt)->getArrivalTime();

    for(auto wpIt = newPickupWpIt ; wpIt != waypoints.end() ; ++wpIt){
        auto arrivalTime = prevArrivalTime + dm.getDistance(prevWpPos, wpIt->getPosition());
        if(wpIt->getDemand() == Demand::DELIVERY){
            // using ideal path
            ttd += arrivalTime - status.getTask(wpIt->getTaskIndex()).idealGoalTime;
            assert(ttd > 0);
        }
        prevWpPos = wpIt->getPosition();
        prevArrivalTime = arrivalTime;
    }

    return ttd;
}

PathWrapper Assignment::extractAndReset() {
    waypoints.clear();
    oldTTD = 0;
    return {index, std::exchange(path, {})};
}

void
Assignment::internalUpdate(const Status &status) {
    std::tie(path, waypoints) = PathFinder::multiAStar(std::move(waypoints), startPos, status, index);
    assert(!status.checkPathWithStatus(path, index));
}

const WaypointsList &Assignment::getWaypoints() const {
    return waypoints;
}

const Path &Assignment::getPath() const {
    return path;
}

int operator<=>(const Assignment &a, const Assignment &b) {
    // signum function
    auto sgn = [](auto val){return (0 < val) - (val < 0);};

    int mcaScore = sgn(a.getMCA() - b.getMCA()) * 4;
    int pathSizeScore = sgn(std::ssize(a.getPath()) - std::ssize(b.getPath())) * 2;
    int idealSpanScore = sgn(a.getIdealGoalTime() - b.getIdealGoalTime());

    return mcaScore + pathSizeScore + idealSpanScore;
}

TimeStep Assignment::computeIdealGoalTime(const Status &status) const{
    assert(waypoints.size() > 0);
    TimeStep igt = 0;
    const auto& dm = status.getDistanceMatrix();

    igt += dm.getDistance(startPos, waypoints.cbegin()->getPosition());

    for(const auto& wp : waypoints){
        if(wp.getDemand() == Demand::DELIVERY) { igt += status.getTask(wp.getTaskIndex()).idealGoalTime; }
    }

    if(waypoints.size() >= 3) {
        igt += dm.getDistance(waypoints.crbegin()->getPosition(), (std::next(waypoints.crbegin()))->getPosition());
    }

    return igt;
}

TimeStep Assignment::getIdealGoalTime() const {
    return idealGoalTime;
}
