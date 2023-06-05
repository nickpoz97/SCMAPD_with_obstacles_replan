#include <array>
#include <cassert>
#include <fstream>

#include "MAPD/Assignment.hpp"
#include "MAPF/PathFinder.hpp"

Assignment::Assignment(const PathWrapper &pW, Status &status) :
    PathWrapper{pW},
    status{status},
    oldTTD{PathWrapper::getTTD()}
{
}

TimeStep Assignment::getMCA() const {
    return getTTD() - oldTTD;
}

bool
Assignment::addTask(int taskId) {

#ifndef NDEBUG
    auto oldWaypointSize = getWaypoints().size();
#endif
    // safe for NoPathException
    auto tmpOldTTD = getTTD();

    auto tmpIdealTTD = insertTaskWaypoints(taskId);

    if(!internalUpdate()){
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
Assignment::internalUpdate() {
    int i = 0;
    // <order, location>
    std::vector<std::pair<int, CompressedCoord>> goals;
    goals.reserve(waypoints.size());
    std::ranges::transform(
        waypoints,
        std::back_inserter(goals),
        [&i](const Waypoint& wp) -> std::pair<int, CompressedCoord> {return {i++, wp.getPosition()};}
    );

    auto resultPath{
        PathFinder::multiAStar(
            goals, getInitialPos(), status.getPaths(getAgentId()), status.getAmbient()
        )
    };
    if(!resultPath){
        return false;
    }

    updatePath(*resultPath);
    updateWaypointsStats();

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

TimeStep Assignment::computeIdealCost() const{
    assert(!getWaypoints().empty());
    TimeStep cost = 0;
    const auto& dm = status.getAmbient().getDistanceMatrix();

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

TimeStep Assignment::computeIdealTTD() const {
    return computeApproxTTD(getWaypoints().cbegin());
}

TimeStep Assignment::computeApproxSpan(WaypointsList::const_iterator startIt) const {
    auto span = 0;
    auto prevPos = startIt == getWaypoints().cbegin() ? getInitialPos() : std::prev(startIt)->getPosition();

    for (auto it = startIt ; it->getDemand() != Demand::END ; ++it){
        auto actualPos = startIt->getPosition();
        span += status.getAmbient().getDistanceMatrix().getDistance(prevPos, actualPos);
        prevPos = actualPos;
    }

    return span;
}

bool
Assignment::removeTasksAndWaypoints(const std::unordered_set<int> &rmvTasksIndices) {
    waypoints.remove_if([&](const Waypoint& wp){
        return wp.getDemand() != Demand::END && rmvTasksIndices.contains(wp.getTaskIndex());}
    );
    std::erase_if(satisfiedTasksIds,[&](int taskId){return rmvTasksIndices.contains(taskId);});

    if(!internalUpdate()){
        return false;
    }

    setIdealTtd(computeIdealTTD());
    oldTTD = getIdealTTD();

    return true;
}

bool Assignment::checkCapacityConstraint() const{
    int actualWeight = 0;

    for(const auto& waypoint : getWaypoints()){
        actualWeight += static_cast<int>(waypoint.getDemand());
        if(actualWeight > getCapacity() || actualWeight < 0){
            return false;
        }
    }
    return true;
}

TimeStep
Assignment::insertTaskWaypoints(int newTaskId) {
    assert(waypoints.crend()->getDemand() == Demand::END);

    const Task& newTask = status.getTask(newTaskId);
    auto pickupWaypoint = getTaskPickupWaypoint(newTask);
    auto deliveryWaypoint = getTaskDeliveryWaypoint(newTask);

    // only end waypoint
    if(waypoints.size() == 1){
        assert(waypoints.cbegin()->getDemand() == Demand::END);
        waypoints.push_front(deliveryWaypoint);
        waypoints.push_front(pickupWaypoint);
        assert(waypoints.crbegin()->getDemand() == Demand::END);

        return computeIdealTTD();
    }

    auto bestPickupIt = waypoints.begin();
    auto bestDeliveryIt = bestPickupIt;

    TimeStep bestApproxTTD = std::numeric_limits<TimeStep>::max();
    TimeStep bestApproxSpan = std::numeric_limits<TimeStep>::max();

    // search for best position for task start and goal
    for(auto wpPickupIt = waypoints.begin() ; wpPickupIt != waypoints.end() ; ++wpPickupIt){
        auto newStartIt = waypoints.insert(wpPickupIt, pickupWaypoint);

        for (auto wpDeliveryIt = wpPickupIt; wpDeliveryIt != waypoints.cend(); ++wpDeliveryIt){
            auto newGoalIt = waypoints.insert(wpDeliveryIt, deliveryWaypoint);

            if(checkCapacityConstraint()){
                auto newApproxTtd = computeApproxTTD(newStartIt);
                std::optional<TimeStep> approxSpan{};
                if(newApproxTtd < bestApproxTTD || newApproxTtd == bestApproxTTD
                                                   && (approxSpan = computeApproxSpan(wpPickupIt)).value() < bestApproxSpan)
                {
                    bestApproxTTD = newApproxTtd;
                    bestPickupIt = wpPickupIt;
                    bestDeliveryIt = wpDeliveryIt;
                    bestApproxSpan = approxSpan.value_or(computeApproxSpan(wpPickupIt));
                }
            }
            assert(waypoints.crbegin()->getDemand() == Demand::END);
            waypoints.erase(newGoalIt);
        }
        waypoints.erase(newStartIt);
    }
    waypoints.insert(bestPickupIt, pickupWaypoint);
    waypoints.insert(bestDeliveryIt, deliveryWaypoint);

    return computeIdealTTD();
}

TimeStep Assignment::computeApproxTTD(WaypointsList::const_iterator newPickupWpIt) const{

    assert(newPickupWpIt != waypoints.end());

    const auto& dm = status.getAmbient().getDistanceMatrix();

    auto ttd = newPickupWpIt == getWaypoints().cbegin() ? 0 : std::prev(newPickupWpIt)->getCumulatedDelay();
    auto prevWpPos = newPickupWpIt == getWaypoints().cbegin() ? getInitialPos() : std::prev(newPickupWpIt)->getPosition();

    auto prevArrivalTime = newPickupWpIt == getWaypoints().cbegin() ?
        0 : std::prev(newPickupWpIt)->getArrivalTime();

    for(auto wpIt = newPickupWpIt ; wpIt != getWaypoints().end() ; ++wpIt){
        auto arrivalTime = prevArrivalTime + dm.getDistance(prevWpPos, wpIt->getPosition());
        if(wpIt->getDemand() == Demand::DELIVERY){
            // using ideal path
            ttd += arrivalTime - status.getTask(wpIt->getTaskIndex()).getIdealGoalTime();
            assert(ttd > 0);
        }
        prevWpPos = wpIt->getPosition();
        prevArrivalTime = arrivalTime;
    }

    assert(!waypoints.empty() );
    return ttd;
}

void Assignment::updateWaypointsStats() {
    TimeStep cumulatedDelay = 0;
    auto wpIt = waypoints.begin();

    for(int t = 0 ; t < std::ssize(getPath()) ; ++t){
        assert(wpIt != waypoints.end());
        assert(!getPath().empty());
        // handling not possible docking
        if(wpIt->getDemand() == Demand::END){
            wpIt->update(std::ssize(getPath()) - 1, cumulatedDelay);
            return;
        }
        if (wpIt->getPosition() == getPath()[t]){
            cumulatedDelay = wpIt->update(t, cumulatedDelay);
            ++wpIt;
        }
    }
}

CompressedCoord Assignment::getInitialPos() const {
    assert(!getPath().empty());

    return *(getPath().cbegin());
}
