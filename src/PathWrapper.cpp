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

bool PWsVector::taskIsSatisfied(int taskId) const {
    return std::ranges::any_of(
        *this,
        [taskId](const PathWrapper& pW){return pW.getSatisfiedTasksIds().contains(taskId);}
    );
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

PathWrapper::PathWrapper(const AgentInfo &agentInfo) :
    index{agentInfo.index},
    capacity{agentInfo.capacity},
    path{agentInfo.startPos},
    idealTTD{0},
    satisfiedTasksIds{},
    waypoints{Waypoint{agentInfo.startPos}}
{}

const WaypointsList &PathWrapper::getWaypoints() const {
    return waypoints;
}

const Path &PathWrapper::getPath() const {
    return path;
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

TimeStep PathWrapper::getIdealTTD() const {
    return idealTTD;
}

bool PathWrapper::empty() const {
    assert(!waypoints.empty());
    assert(!(waypoints.size() == 1) || waypoints.cbegin()->getDemand() == Demand::END);
    assert(
        !(waypoints.size() > 1) ||
            (waypoints.size() == satisfiedTasksIds.size() * 2 + 1 && waypoints.cbegin()->getDemand() == Demand::END)
    );
    return getWaypoints().size() == 1;
}

int PathWrapper::getAgentId() const {
    return index;
}

[[maybe_unused]] int PathWrapper::getCapacity() const {
    return capacity;
}

void PathWrapper::updatePath(const Path &newPath, TimeStep connectionTimeStep) {
    // remove old path
    path.erase(path.begin() + connectionTimeStep, path.end());

    // copy the new one
    path.reserve(path.size() + newPath.size());
    path.insert(path.begin() + connectionTimeStep, newPath.cbegin(), newPath.cend());
}

void PathWrapper::setIdealTtd(TimeStep idealTtd) {
    this->idealTTD = idealTtd;
}

bool PathWrapper::isAvailable(TimeStep t) const {
    return t >= std::ssize(path) - 1;
}

void PathWrapper::extendAndReset(TimeStep actualTimeStep){
    assert(!path.empty());
    assert(!waypoints.empty() && waypoints.crend()->getDemand() == Demand::END);

    auto lastPos = *path.crbegin();
    for(int t = std::ssize(path); t <= actualTimeStep ; ++t){
        path.push_back(lastPos);
    }

    waypoints.clear();
    waypoints.emplace_back(lastPos);

    assert(waypoints.size() == 1 && waypoints.cbegin()->getDemand() == Demand::END);

    idealTTD = 0;
}

