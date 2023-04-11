#include <array>
#include <cassert>
#include <fstream>

#include "Assignment.hpp"
#include "MAPF/PathFinder.hpp"

Assignment::Assignment(const AgentInfo &agentInfo) :
    PathWrapper{agentInfo},
    oldTTD{0}
{}

Assignment::Assignment(const PathWrapper& pW) :
    PathWrapper{pW},
    oldTTD{PathWrapper::getTTD()}
{}

TimeStep Assignment::getMCA() const {
    return getTTD() - oldTTD;
}

bool
Assignment::addTask(int taskId, const Status &status) {

#ifndef NDEBUG
    auto oldWaypointSize = getWaypoints().size();
#endif
    // safe for NoPathException
    auto tmpOldTTD = getTTD();

    auto tmpIdealTTD = insertTaskWaypoints(
        status.getTask(taskId),
        status.getDistanceMatrix(),
        status.getTasks(),
        getCapacity()
    );

    if(!internalUpdate(status)){
        return false;
    }

    oldTTD = tmpOldTTD;
    setIdealTtd(tmpIdealTTD);

    satisfiedTasksIds.emplace(taskId);
#ifndef NDEBUG
    assert(oldWaypointSize == getWaypoints().size() - 2);
    int sum = 0;
    for(const auto& w : getWaypoints()){
        sum += static_cast<int>(w.getDemand());
    }
    assert(sum == 0);
    assert(
        std::ranges::all_of(
            getWaypoints().cbegin(),
            getWaypoints().cend(),
            [&](const Waypoint& wp){
                return wp.getDemand() == Demand::END || satisfiedTasksIds.contains(wp.getTaskIndex());
            }
        )
    );
    assert(
        std::ranges::all_of(
            getWaypoints().cbegin(),
            getWaypoints().cend(),
            [&](const Waypoint& wp){
                return std::find(getPath().cbegin(), getPath().cend(), wp.getPosition()) != getPath().cend();
            }
        )
    );
#endif

    return true;
}

bool
Assignment::internalUpdate(const Status &status) {
    auto resultPath{PathFinder::multiAStar(getWaypoints(), getInitialPos(), status, getAgentId())};
    if(!resultPath){
        return false;
    }

    setPath(std::move(*resultPath));
    updateWaypointsStats(status.getTasks());

    oldTTD = status.getPathWrappers().getTasksDelay(getAgentId());

    assert(!status.hasIllegalPositions(getPath()));
    assert(!status.checkPathWithStatus(getPath(), getAgentId()));
    return true;
}

int operator<=>(const Assignment &a, const Assignment &b) {
    // signum function
    auto sgn = [](auto val){return (0 < val) - (val < 0);};

    int mcaScore = sgn(a.getMCA() - b.getMCA());
    int pathSizeScore = sgn(a.getLastDeliveryTimeStep() - b.getLastDeliveryTimeStep());
    int idealCost = sgn(a.getIdealTTD() - b.getIdealTTD());

    return mcaScore * 4 + pathSizeScore * 2 + idealCost;
}

TimeStep Assignment::computeIdealCost(const Status &status) const{
    assert(!getWaypoints().empty());
    TimeStep cost = 0;
    const auto& dm = status.getDistanceMatrix();

    cost += dm.getDistance(getInitialPos(), getWaypoints().cbegin()->getPosition());
    auto prevPos = getInitialPos();
    for(const auto& wp: getWaypoints()){
        if(wp.getDemand() == Demand::END){
            break;
        }
        auto actualPos = wp.getPosition();
        cost += dm.getDistance(prevPos, actualPos);
        prevPos = actualPos;
    }

    return cost;
}

TimeStep Assignment::computeIdealTTD(const DistanceMatrix &dm, const std::vector<Task> &tasks) const {
    return computeApproxTTD(dm, tasks, getWaypoints().cbegin());
}

static void updateWaypointsStats(WaypointsList& waypointsList, const Path& path, const std::vector<Task>& tasksVector){
    TimeStep cumulatedDelay = 0;
    auto wpIt = waypointsList.begin();

    for(int t = 0 ; t < std::ssize(path) ; ++t){
        assert(wpIt != waypointsList.end());
        assert(!path.empty());
        // handling not possible docking
        if(wpIt->getDemand() == Demand::END){
            wpIt->update(std::ssize(path) - 1, tasksVector, cumulatedDelay);
            return;
        }
        if (wpIt->getPosition() == path[t]){
            cumulatedDelay = wpIt->update(t, tasksVector, cumulatedDelay);
            ++wpIt;
        }
    }
}

TimeStep Assignment::computeApproxSpan(const DistanceMatrix &dm, WaypointsList::const_iterator startIt) const {
    auto span = 0;
    auto prevPos = startIt == getWaypoints().cbegin() ? getInitialPos() : std::prev(startIt)->getPosition();

    for (auto it = startIt ; it->getDemand() != Demand::END ; ++it){
        auto actualPos = startIt->getPosition();
        span += dm.getDistance(prevPos, actualPos);
        prevPos = actualPos;
    }

    return span;
}

bool
Assignment::removeTasksAndWaypoints(const std::unordered_set<int> &rmvTasksIndices, const Status& status) {
    waypoints.remove_if([&](const Waypoint& wp){
        return wp.getDemand() != Demand::END && rmvTasksIndices.contains(wp.getTaskIndex());}
    );
    std::erase_if(satisfiedTasksIds,[&](int taskId){return rmvTasksIndices.contains(taskId);});

    if(!internalUpdate(status)){
        return false;
    }

    const auto& dm = status.getDistanceMatrix();
    const auto& tasks = status.getTasks();

    setIdealTtd(computeIdealTTD(dm, tasks));
    oldTTD = getIdealTTD();

    return true;
}

bool Assignment::checkCapacityConstraint(int capacity) const{
    int actualWeight = 0;

    for(const auto& waypoint : getWaypoints()){
        actualWeight += static_cast<int>(waypoint.getDemand());
        if(actualWeight > capacity || actualWeight < 0){
            return false;
        }
    }
    return true;
}

TimeStep
Assignment::insertTaskWaypoints(const Task &newTask, const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                 int agentCapacity) {
    assert(waypoints.crend()->getDemand() == Demand::END);

    // only end waypoint
    if(waypoints.size() == 1){
        assert(waypoints.cbegin()->getDemand() == Demand::END);
        waypoints.push_front(getTaskDeliveryWaypoint(newTask));
        waypoints.push_front(getTaskPickupWaypoint(newTask));
        assert(waypoints.crbegin()->getDemand() == Demand::END);
        return computeIdealTTD(dm, tasksVector);
    }

    auto bestPickupIt = waypoints.begin();
    auto bestDeliveryIt = bestPickupIt;

    TimeStep bestApproxTTD = std::numeric_limits<TimeStep>::max();
    TimeStep bestApproxSpan = std::numeric_limits<TimeStep>::max();

    // search for best position for task start and goal
    for(auto wpPickupIt = waypoints.begin(); wpPickupIt != waypoints.end() ; ++wpPickupIt){
        for (auto wpDeliveryIt = wpPickupIt; wpDeliveryIt != waypoints.cend(); ++wpDeliveryIt){
            auto [newStartIt, newGoalIt] = insertNewWaypoints(newTask, wpPickupIt, wpDeliveryIt);
            if(checkCapacityConstraint(agentCapacity)){
                auto newApproxTtd = computeApproxTTD(dm, tasksVector, newStartIt);
                std::optional<TimeStep> approxSpan{};
                if(newApproxTtd < bestApproxTTD || newApproxTtd == bestApproxTTD
                                                   && (approxSpan = computeApproxSpan(dm, wpPickupIt)).value() < bestApproxSpan)
                {
                    bestApproxTTD = newApproxTtd;
                    bestPickupIt = wpPickupIt;
                    bestDeliveryIt = wpDeliveryIt;
                    bestApproxSpan = approxSpan.value_or(computeApproxSpan(dm, wpPickupIt));
                }
            }
            assert(waypoints.crbegin()->getDemand() == Demand::END);
            restorePreviousWaypoints(newStartIt, newGoalIt);
        }
    }
    insertNewWaypoints(newTask, bestPickupIt, bestDeliveryIt);

    return computeIdealTTD(dm, tasksVector);
}

void Assignment::restorePreviousWaypoints(WaypointsList::iterator waypointStart,
                                           WaypointsList::iterator waypointGoal) {
    waypoints.erase(waypointStart);
    waypoints.erase(waypointGoal);
}

std::pair<WaypointsList::iterator, WaypointsList::iterator> Assignment::insertNewWaypoints(const Task &task, WaypointsList::iterator waypointStart,
                                                                                            WaypointsList::iterator waypointGoal) {
    return {
            waypoints.insert(waypointStart, getTaskPickupWaypoint(task)),
            waypoints.insert(waypointGoal, getTaskDeliveryWaypoint(task))
    };
}

TimeStep Assignment::computeApproxTTD(const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                       WaypointsList::const_iterator newPickupWpIt) const{

    assert(newPickupWpIt != waypoints.end());

    auto ttd = newPickupWpIt == getWaypoints().cbegin() ? 0 : std::prev(newPickupWpIt)->getCumulatedDelay();
    auto prevWpPos = newPickupWpIt == getWaypoints().cbegin() ? getInitialPos() : std::prev(newPickupWpIt)->getPosition();
    auto prevArrivalTime = newPickupWpIt == getWaypoints().cbegin() ? 0 : std::prev(newPickupWpIt)->getArrivalTime();

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

    assert(!(waypoints.size() == 3) || ttd == dm.getDistance(getInitialPos(), newPickupWpIt->getPosition()));
    return ttd;
}

void Assignment::updateWaypointsStats(const std::vector<Task>& tasksVector){
    TimeStep cumulatedDelay = 0;
    auto wpIt = waypoints.begin();

    for(int t = 0 ; t < std::ssize(getPath()) ; ++t){
        assert(wpIt != waypoints.end());
        assert(!getPath().empty());
        // handling not possible docking
        if(wpIt->getDemand() == Demand::END){
            wpIt->update(std::ssize(getPath()) - 1, tasksVector, cumulatedDelay);
            return;
        }
        if (wpIt->getPosition() == getPath()[t]){
            cumulatedDelay = wpIt->update(t, tasksVector, cumulatedDelay);
            ++wpIt;
        }
    }
}
