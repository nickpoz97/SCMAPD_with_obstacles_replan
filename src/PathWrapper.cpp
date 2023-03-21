#include "MAPF/PathFinder.hpp"
#include "Assignment.hpp"
#include <set>
#include <fstream>
#include <cassert>
#include <array>
#include <PathWrapper.hpp>
#include <utility>
#include <random>

TimeStep PWsVector::getMaxSpanCost() const {
    return std::max_element(
            cbegin(),
            cend(),
            [](const PathWrapper &pWA, const PathWrapper &pWB) {
                return pWA.getLastDeliveryTimeStep() <
                       pWB.getLastDeliveryTimeStep();
            }
    )->getLastDeliveryTimeStep();
}

TimeStep PWsVector::getTTD() const {
    return std::accumulate(
            cbegin(),
            cend(),
            0,
            [](TimeStep ttd, const PathWrapper& pW) {return ttd + pW.getTTD();}
    );
}

TimeStep PWsVector::getTTT() const {
    return std::accumulate(
            cbegin(),
            cend(),
            0,
            [](TimeStep ttt, const PathWrapper& pW) {return ttt + pW.getLastDeliveryTimeStep();}
    );
}

TimeStep PWsVector::getSpan(int agentId) const {
    return operator[](agentId).getLastDeliveryTimeStep();
}

TimeStep PWsVector::getTasksDelay(int agentId) const {
    return operator[](agentId).getTTD();
}

const Path& PWsVector::getPath(int agentId) const {
    return operator[](agentId).getPath();
}

bool PathWrapper::removeTasksAndWaypoints(const std::unordered_set<int> &rmvTasksIndices) {
    waypoints.remove_if([&](const Waypoint& wp){
        return wp.getDemand() != Demand::END && rmvTasksIndices.contains(wp.getTaskIndex());}
    );
    return std::erase_if(satisfiedTasksIds,[&](int taskId){return rmvTasksIndices.contains(taskId);});
}

TimeStep PathWrapper::getTTD() const {
    return waypoints.crbegin()->getCumulatedDelay();
}

TimeStep PathWrapper::getLastDeliveryTimeStep() const {
    if(waypoints.size() <= 1){
        return 0;
    }
    return std::next(waypoints.crbegin())->getArrivalTime();
}

const std::unordered_set<int> &PathWrapper::getSatisfiedTasksIds() const {
    return satisfiedTasksIds;
}

PathWrapper::PathWrapper(Path path, WaypointsList  wpList, std::unordered_set<int> satisfiedTasksIds) :
        path{std::move(path)},
        waypoints{std::move(wpList)},
        satisfiedTasksIds{std::move(satisfiedTasksIds)}
    {}

const WaypointsList &PathWrapper::getWaypoints() const {
    return waypoints;
}

const Path &PathWrapper::getPath() const {
    return path;
}

CompressedCoord PathWrapper::getInitialPos() const {
    assert(!path.empty());
    return path[0];
}

void PathWrapper::pathAndWaypointsUpdate(std::pair<Path, WaypointsList> &&updatedData) {
    path = std::move(updatedData.first);
    waypoints = std::move(updatedData.second);
}

void
PathWrapper::insertTaskWaypoints(const Task &newTask, const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                int agentCapacity) {
    assert(waypoints.crend()->getDemand() == Demand::END);

    if(waypoints.size() == 1){
        waypoints.push_front(getTaskDeliveryWaypoint(newTask));
        waypoints.push_front(getTaskPickupWaypoint(newTask));
        return;
    }

    auto bestPickupIt = waypoints.begin();
    auto bestDeliveryIt = bestPickupIt;

    TimeStep bestApproxTTD = std::numeric_limits<decltype(bestApproxTTD)>::max();

    // search for best position for task start and goal
    for(auto wpPickupIt = waypoints.begin(); wpPickupIt != waypoints.end() ; ++wpPickupIt){
        for (auto wpDeliveryIt = wpPickupIt; wpDeliveryIt != waypoints.cend(); ++wpDeliveryIt){
            auto [newStartIt, newGoalIt] = insertNewWaypoints(newTask, wpPickupIt, wpDeliveryIt);
            if(checkCapacityConstraint(agentCapacity)){
                auto newApproxTtd = computeApproxTTD(dm, tasksVector, newStartIt);
                if(newApproxTtd < bestApproxTTD){
                    bestApproxTTD = newApproxTtd;
                    bestPickupIt = wpPickupIt;
                    bestDeliveryIt = wpDeliveryIt;
                }
            }
            restorePreviousWaypoints(newStartIt, newGoalIt);
        }
    }
    insertNewWaypoints(newTask, bestPickupIt, bestDeliveryIt);
}

void PathWrapper::restorePreviousWaypoints(WaypointsList::iterator waypointStart,
                                           WaypointsList::iterator waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

std::pair<WaypointsList::iterator, WaypointsList::iterator> PathWrapper::insertNewWaypoints(const Task &task, WaypointsList::iterator waypointStart,
                                                                                            WaypointsList::iterator waypointGoal) {
    return {
            waypoints.insert(waypointStart, getTaskPickupWaypoint(task)),
            waypoints.insert(waypointGoal, getTaskDeliveryWaypoint(task))
    };
}

TimeStep PathWrapper::computeApproxTTD(const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                      WaypointsList::iterator newPickupWpIt) const{

    assert(newPickupWpIt != waypoints.end());

    auto ttd = newPickupWpIt == getWaypoints().begin() ? 0 : std::prev(newPickupWpIt)->getCumulatedDelay();
    auto prevWpPos = newPickupWpIt == getWaypoints().begin() ? getInitialPos() : std::prev(newPickupWpIt)->getPosition();
    auto prevArrivalTime = newPickupWpIt == getWaypoints().begin() ? 0 : std::prev(newPickupWpIt)->getArrivalTime();

    for(auto wpIt = newPickupWpIt ; wpIt != getWaypoints().end() ; ++wpIt){
        auto arrivalTime = prevArrivalTime + dm.getDistance(prevWpPos, wpIt->getPosition());
        if(wpIt->getDemand() == Demand::DELIVERY){
            // using ideal path
            ttd += arrivalTime - tasksVector[wpIt->getTaskIndex()].idealGoalTime;
            assert(ttd > 0);
        }
        prevWpPos = wpIt->getPosition();
        prevArrivalTime = arrivalTime;
    }

    return ttd;
}

bool PathWrapper::checkCapacityConstraint(int capacity) const{
    int actualWeight = 0;

    for(const auto& waypoint : getWaypoints()){
        actualWeight += static_cast<int>(waypoint.getDemand());
        if(actualWeight > capacity || actualWeight < 0){
            return false;
        }
    }
    return true;
}

int PathWrapper::randomTaskId(int magicNumber) const {
    assert(!satisfiedTasksIds.empty());
    std::vector<int> shuffled_tasks{satisfiedTasksIds.cbegin(), satisfiedTasksIds.cend()};

    auto seed = hash_value(*this);
    boost::hash_combine(seed, magicNumber);

    // shuffle them using status hash as seed
    std::shuffle(
        shuffled_tasks.begin(),
        shuffled_tasks.end(),
        std::default_random_engine(seed)
    );

    return *shuffled_tasks.begin();
}

TimeStep PathWrapper::getIdealCost() const {
    return idealCost;
}

TimeStep PWsVector::getIdealCost() const {
    return std::accumulate(cbegin(), cend(), 0, [](TimeStep sum, const PathWrapper& pW){
        return sum += pW.getIdealCost();
    });
}

TimeStep PathWrapper::getActualTTD() const{
    assert(!getWaypoints().empty());
    return getWaypoints().crbegin()->getCumulatedDelay();
}