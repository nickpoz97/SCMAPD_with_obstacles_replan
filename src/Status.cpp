//
// Created by nicco on 05/12/2022.
//

#include <algorithm>
#include <fmt/core.h>
#include <queue>
#include <random>

#include "Status.hpp"

Status::Status(AmbientMap &&ambientMap, const std::vector<AgentInfo> &agents, std::vector<Task> &&tasks,
               PathfindingStrategy strategy) :
        ambient(std::move(ambientMap)),
        tasksVector(std::move(tasks)),
        pathsWrappers{initializePathsWrappers(agents)},
        pathFindingStrategy{strategy}
        {}

const Task & Status::getTask(int i) const {
    return tasksVector[i];
}

const std::vector<Task> &Status::getTasks() const {
    return tasksVector;
}

std::pair<int, int> Status::update(ExtractedPath &&extractedPath) {
    auto agentId = extractedPath.agentId;
    auto taskId = extractedPath.newTaskId;

    longestPathSize = std::max(static_cast<int>(extractedPath.wrapper.getPath().size()), longestPathSize);
    pathsWrappers[agentId] = std::move(extractedPath.wrapper);

    return {agentId, taskId};
}

std::vector<CompressedCoord>
Status::getValidNeighbors(int agentId, CompressedCoord c, TimeStep t, bool includeHoldAction) const {
    std::vector<CompressedCoord> neighbors;
    neighbors.reserve(AmbientMap::nDirections);

    for(int i = 0 ; i < AmbientMap::nDirections ; ++i){
        if(!includeHoldAction && i == AmbientMap::getHoldDirectionIndex()){
            continue;
        }
        auto result = ambient.movement(c, i);
        if(result.has_value() && !checkDynamicObstacle(agentId, c, result.value(), t)){
            neighbors.push_back(result.value());
        }
    }

    return neighbors;
}

CompressedCoord Status::holdOrAvailablePos(int agentId, CompressedCoord c, TimeStep t) const{
    // first check if agent can hold
    if(checkDynamicObstacle(agentId, c, c, t)){
        auto neighbors = getValidNeighbors(agentId, c, t, false);
        if(!neighbors.empty()){
            return neighbors[0];
        }
        throw std::runtime_error(fmt::format("Agent {} cannot move nor hold at timestep {}", agentId, t));
    }
    return c;
}

bool Status::checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2, TimeStep t1) const{
    assert(agentId >= 0 && agentId < getNAgents());

    auto predicate = [t1, coord1, coord2](const PathWrapper& pW){
        const auto& p = pW.getPath();

        // if path is empty there are no conflicts
        if(p.empty()){
            return false;
        }
        auto t2 = std::min(t1 + 1, static_cast<int>(p.size()-1));

        // todo check this
        bool nodeConflict = coord2 == p[t2];
        bool edgeConflict = t1 < (p.size() - 1) && coord1 == p[t2] && coord2 == p[t1];
        bool baseConflict = coord2 == *p.cbegin();

        return nodeConflict || edgeConflict || baseConflict;
    };

    return std::ranges::any_of(pathsWrappers.begin(), pathsWrappers.begin() + agentId, predicate) ||
        std::ranges::any_of(pathsWrappers.begin() + agentId + 1, pathsWrappers.end(), predicate);
}

const DistanceMatrix& Status::getDistanceMatrix() const{
    return ambient.getDistanceMatrix();
}

bool Status::checkPathWithStatus(const Path &path, int agentId) const{
    auto predicate = [&](const PathWrapper& other){return checkPathConflicts(path, other.getPath());};

    return std::ranges::any_of(
        pathsWrappers.begin(),
        pathsWrappers.begin() + agentId,
        predicate
    ) ||
    std::ranges::any_of(
        pathsWrappers.begin() + agentId + 1,
        pathsWrappers.end(),
        predicate
    );
}

bool Status::checkAllConflicts() const {
    for(int i = 0 ; i < pathsWrappers.size() ; ++i){
        for(int j = i+1 ; j < pathsWrappers.size() ; ++j){
            if(checkPathConflicts(i, j)){
                return true;
            }
        }
    }
    return false;
}

bool Status::checkPathConflicts(int i, int j) const{
    if(i == j){
        return false;
    }

    return checkPathConflicts(pathsWrappers[i].getPath(), pathsWrappers[j].getPath());
}

bool Status::checkPathConflicts(const Path &pA, const Path &pB) {
    if(pA.empty() || pB.empty()){
        return false;
    }

    for(int t = 0 ; t < std::max(pA.size(), pB.size()) ; ++t) {
        bool nodeConflict =
                pA[std::min(t, static_cast<int>(pA.size() - 1))] == pB[std::min(t, static_cast<int>(pB.size() - 1))];
        bool edgeConflict = t < static_cast<int>(std::min(pA.size(), pB.size())) - 1 && pA[t] == pB[t + 1] && pA[t + 1] == pB[t];

        if(nodeConflict || edgeConflict){
            return true;
        }
    }
    return false;
}

TimeStep Status::getLongestPathSize() const {
    return longestPathSize;
}

std::vector<PathWrapper> Status::initializePathsWrappers(const std::vector<AgentInfo> &agents) {
    std::vector<PathWrapper> pWs{};
    pWs.reserve(agents.size());

    std::ranges::transform(
        agents,
        std::back_inserter(pWs),
        [](const AgentInfo& a) -> PathWrapper {
            return {
                {a.startPos},{},{}
            };
        }
    );
    return pWs;
}

bool Status::hasIllegalPositions(const Path& path) const{
    const auto& dm = ambient.getDistanceMatrix();
    return std::ranges::any_of(path, [&](CompressedCoord cc){return !ambient.isValid(dm.from1Dto2D(cc));});
}

std::optional<int> Status::getMaxPosVisits() const{
    switch (pathFindingStrategy) {
        case PathfindingStrategy::EAGER:
            return 2;
        case PathfindingStrategy::FORWARD_ONLY:
            return 1;
        case PathfindingStrategy::LAZY:
            return static_cast<int>(AmbientMap::nDirections);
        default:
            return std::nullopt;
    }
}

int Status::getNAgents() const {
    return static_cast<int>(pathsWrappers.size());
}

const PWsVector & Status::getPathWrappers() const {
    return pathsWrappers;
}

PathWrapper &Status::getPathWrapper(int agentId) {
    return pathsWrappers[agentId];
}

void Status::setPathWrappers(PWsVector &&other) {
    pathsWrappers = std::move(other);
}

VerbosePath Status::toVerbosePath(int i) const {
    assert(i >= 0 && i < pathsWrappers.size());
    const auto& path = pathsWrappers[i].getPath();

    VerbosePath vP;
    vP.reserve(path.size());
    std::ranges::transform(
        path,
        std::back_inserter(vP),
        [this](const CompressedCoord& cc){return getDistanceMatrix().from1Dto2D(cc);}
    );

    return vP;
}

std::unordered_set<int> Status::chooseNWorstTasks(int n, Objective obj) const {
    // taskId, value
    using TaskInfo = std::pair<int, TimeStep>;

    n = std::min(static_cast<int>(tasksVector.size()), n);

    std::vector<TaskInfo> orderedTasks;
    orderedTasks.reserve(tasksVector.size());

    for (const auto& pW : pathsWrappers){
        for (const auto& wp : pW.getWaypoints()){
            if(wp.getDemand() != Demand::DELIVERY){
                continue;
            }
            switch (obj) {
                case Objective::MAKESPAN:
                    orderedTasks.emplace_back(wp.getTaskIndex(), wp.getArrivalTime());
                    break;
                case Objective::TTD:
                    orderedTasks.emplace_back(wp.getTaskIndex(), wp.getDelay(tasksVector));
                    break;
            }
        }
    }

    std::sort(
            orderedTasks.begin(),
            orderedTasks.end(),
            [](const TaskInfo& ta, const TaskInfo& tb){return ta.second > tb.second;}
    );

    std::unordered_set<int> taskIndicesToRemove{};
    taskIndicesToRemove.reserve(n);
    for(int i = 0 ; i < n ; ++i){
        taskIndicesToRemove.insert(orderedTasks[i].first);
    }
    return taskIndicesToRemove;
}

std::unordered_set<int> Status::chooseNRandomTasks(int iterIndex, int n) const{
    n = std::min(static_cast<int>(tasksVector.size()), n);

    std::vector<int> shuffled_tasks{};
    shuffled_tasks.reserve(tasksVector.size());

    // copy indices
    std::ranges::transform(
            tasksVector,
            std::back_inserter(shuffled_tasks),
            [](const Task& t){return t.index;}
    );

    auto seed = hash_value(*this);
    boost::hash_combine(seed, iterIndex);

    // shuffle them using status hash as seed
    std::shuffle(
            shuffled_tasks.begin(),
            shuffled_tasks.end(),
            std::default_random_engine(seed)
    );
    return {shuffled_tasks.cbegin(), shuffled_tasks.cbegin() + n};
}

std::unordered_set<int> Status::chooseTasksFromNWorstAgents(int iterIndex, int n, Objective obj) const {
    // agentId, value
    using AgentInfo = std::pair<int, TimeStep>;

    n = std::min(static_cast<int>(pathsWrappers.size()), n);

    std::vector<AgentInfo> orderedAgents;
    orderedAgents.reserve(orderedAgents.size());

    for (int i = 0 ; i < pathsWrappers.size() ; ++i){
        const auto& pW = pathsWrappers[i];

        switch (obj) {
            case Objective::MAKESPAN:
                orderedAgents.emplace_back(i, pW.getLastDeliveryTimeStep());
                break;
            case Objective::TTD:
                orderedAgents.emplace_back(i, pW.getTTD());
                break;
        }
    }

    std::sort(
        orderedAgents.begin(),
        orderedAgents.end(),
        [](const AgentInfo & ta, const AgentInfo& tb){return ta.second > tb.second;}
    );

    std::unordered_set<int> taskIndicesToRemove{};
    taskIndicesToRemove.reserve(n);

    for(int i = 0 ; i < n ; ++i){
        const auto& pW = pathsWrappers[orderedAgents[i].first];
        if(!pW.getSatisfiedTasksIds().empty()) {
            taskIndicesToRemove.insert(pW.randomTaskId(iterIndex));
        }
    }
    return taskIndicesToRemove;
}
