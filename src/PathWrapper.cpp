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

TimeStep
PathWrapper::insertTaskWaypoints(const Task &newTask, const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                int agentCapacity) {
    assert(waypoints.crend()->getDemand() == Demand::END);

    // only end waypoint
    if(waypoints.size() == 1){
        assert(waypoints.cbegin()->getDemand() == Demand::END);
        waypoints.push_front(getTaskDeliveryWaypoint(newTask));
        waypoints.push_front(getTaskPickupWaypoint(newTask));
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
            restorePreviousWaypoints(newStartIt, newGoalIt);
        }
    }
    insertNewWaypoints(newTask, bestPickupIt, bestDeliveryIt);

    return computeIdealTTD(dm, tasksVector);
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
                                      WaypointsList::const_iterator newPickupWpIt) const{

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

    assert(!(waypoints.size() == 3) || ttd == dm.getDistance(getInitialPos(), newPickupWpIt));
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

TimeStep PathWrapper::computeApproxSpan(const DistanceMatrix &dm, WaypointsList::const_iterator startIt) const {
    auto span = 0;
    auto prevPos = startIt == waypoints.cbegin() ? getInitialPos() : std::prev(startIt)->getPosition();

    for (auto it = startIt ; it->getDemand() != Demand::END ; ++it){
        auto actualPos = startIt->getPosition();
        span += dm.getDistance(prevPos, actualPos);
        prevPos = actualPos;
    }

    return span;
}

TimeStep PathWrapper::getIdealTTD() const {
    return idealTTD;
}

TimeStep PathWrapper::computeIdealTTD(const DistanceMatrix &dm, const std::vector<Task> &tasks) const {
    return computeApproxTTD(dm, tasks, waypoints.cbegin());
}

TimeStep PWsVector::getIdealCost() const {
    return std::accumulate(cbegin(), cend(), 0, [](TimeStep sum, const PathWrapper& pW){
        return sum + pW.getIdealCost();
    });
}

TimeStep PWsVector::getRelativeTTD() const {
    return std::accumulate(cbegin(), cend(), 0, [](TimeStep sum, const PathWrapper& pW){
        return sum + pW.getTTD() - pW.getIdealTTD();
    });
}
