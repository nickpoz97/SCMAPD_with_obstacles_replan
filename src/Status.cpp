//
// Created by nicco on 05/12/2022.
//

#include <algorithm>
#include <fmt/core.h>
#include <queue>
#include <random>

#include "Status.hpp"

Status::Status(AmbientMap ambientMap, const std::vector<AgentInfo> &agents, bool noConflicts) :
    ambient(std::move(ambientMap)),
    pathsWrappers{initializePathsWrappers(agents)},
    noConflicts{noConflicts}
    {}

const Task & Status::getTask(int i) const {
    assert(notAssignedTasks.contains(i) || assignedTasks.contains(i));
    const auto result = notAssignedTasks.find(i);
    return result != notAssignedTasks.cend() ? result->second : assignedTasks.find(i)->second;
}

std::pair<int, int> Status::update(ExtractedPath extractedPath) {
    auto agentId = extractedPath.agentId;
    auto taskId = extractedPath.newTaskId;

    assert(!assignedTasks.contains(taskId));
    assignedTasks.emplace(taskId, notAssignedTasks.at(taskId));
    // task has just been assigned
    notAssignedTasks.erase(taskId);

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
        if(result.has_value() &&
            (noConflicts || !checkDynamicObstacle(agentId, c, *result, t))){
            neighbors.push_back(result.value());
        }
    }

    return neighbors;
}

bool Status::checkDynamicObstacle(int agentId, CompressedCoord coord1, CompressedCoord coord2,
                                  TimeStep t1) const{
    assert(agentId >= 0 && agentId < getNAgents());

    auto predicate = [t1, coord1, coord2](const PathWrapper& pW){
        const auto& p = pW.getPath();

        // if path is empty there are no conflicts
        if(p.empty()){
            return false;
        }
        auto t2 = std::min(t1 + 1, static_cast<int>(p.size()-1));
        assert(t2 >= 0);

        bool nodeConflict = coord2 == p[t2];
        bool edgeConflict = t1 < (p.size() - 1) && coord1 == p[t2] && coord2 == p[t1];

        return nodeConflict || edgeConflict;
    };

    return std::ranges::any_of(pathsWrappers.begin(), pathsWrappers.begin() + agentId, predicate) ||
        std::ranges::any_of(pathsWrappers.begin() + agentId + 1, pathsWrappers.end(), predicate);
}

const DistanceMatrix& Status::getDistanceMatrix() const{
    return ambient.getDistanceMatrix();
}

bool Status::checkPathWithStatus(const Path &path, int agentId) const{
    if(noConflicts){
        return false;
    }

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
    if(noConflicts){
        return false;
    }

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
        bool edgeConflict = t < std::min(std::ssize(pA), std::ssize(pB)) - 1 && pA[t] == pB[t + 1] && pA[t + 1] == pB[t];

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
            return PathWrapper{a};
        }
    );
    return pWs;
}

bool Status::hasIllegalPositions(const Path& path) const{
    const auto& dm = ambient.getDistanceMatrix();
    return std::ranges::any_of(path, [&](CompressedCoord cc){return !ambient.isValid(dm.from1Dto2D(cc));});
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

std::unordered_set<int> Status::chooseNWorstTasks(int n, Metric mt) const {
    // taskId, value
    using TaskInfo = std::pair<int, TimeStep>;

    n = std::min(static_cast<int>(assignedTasks.size()), n);

    std::vector<TaskInfo> orderedTasks;
    orderedTasks.reserve(assignedTasks.size());

    for (const auto& pW : pathsWrappers){
        for (const auto& wp : pW.getWaypoints()){
            if(wp.getDemand() != Demand::DELIVERY){
                continue;
            }
            switch (mt) {
                case Metric::ARRIVAL_TIME:
                    orderedTasks.emplace_back(wp.getTaskIndex(), wp.getArrivalTime());
                    break;
                case Metric::DELAY:
                    orderedTasks.emplace_back(wp.getTaskIndex(), wp.getDelay());
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

    for(int i = 0 ; i < n ; ++i){
        taskIndicesToRemove.insert(orderedTasks[i].first);
    }
    return taskIndicesToRemove;
}

std::unordered_set<int> Status::chooseNRandomTasks(int iterIndex, int n) const{
    n = std::min(static_cast<int>(assignedTasks.size()), n);

    std::vector<int> shuffled_tasks{};
    shuffled_tasks.reserve(assignedTasks.size());

    // copy indices
    std::ranges::transform(
            assignedTasks,
            std::back_inserter(shuffled_tasks),
            [](const std::pair<int, Task>& item){return item.first;}
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

std::unordered_set<int> Status::chooseTasksFromNWorstAgents(int iterIndex, int n, Metric mt) const {
    // agentId, value
    using AgentInfo = std::pair<int, TimeStep>;

    n = std::min(static_cast<int>(pathsWrappers.size()), n);

    std::vector<AgentInfo> orderedAgents;
    orderedAgents.reserve(orderedAgents.size());

    for (int i = 0 ; i < pathsWrappers.size() ; ++i){
        const auto& pW = pathsWrappers[i];

        switch (mt) {
            case Metric::ARRIVAL_TIME:
                orderedAgents.emplace_back(i, pW.getLastDeliveryTimeStep());
                break;
            case Metric::DELAY:
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

    for(int i = 0 ; i < n ; ++i){
        const auto& pW = pathsWrappers[orderedAgents[i].first];
        if(!pW.getSatisfiedTasksIds().empty()) {
            taskIndicesToRemove.insert(pW.randomTaskId(iterIndex));
        }
    }
    return taskIndicesToRemove;
}

std::string
Status::getAgentsSnapshot(int agentId, TimeStep t, CompressedCoord actual) const {
    auto rowStrings = ambient.getRowsStrings();
    auto dm = ambient.getDistanceMatrix();

    for (int i = 0 ; i < pathsWrappers.size() ; ++i){
        if(i == agentId){
            continue;
        }
        const auto& path = pathsWrappers.getPath(i);
        auto coord2D = dm.from1Dto2D(path[std::min(t, static_cast<int>(path.size()-1))]);

        // check if agents are not in illegal positions
        assert(ambient.isValid(coord2D));

        // check if another agent is present
        assert(rowStrings[coord2D.row][coord2D.col] != 'o');

        rowStrings[coord2D.row][coord2D.col] = 'o';
    }

    auto actual2D = dm.from1Dto2D(actual);
    // check if agents are not in illegal positions
    assert(ambient.isValid(actual2D));
    // check if another agent is present
    assert(rowStrings[actual2D.row][actual2D.col] != 'o');
    rowStrings[actual2D.row][actual2D.col] = 'a';

    std::string gridString;
    gridString.reserve(ambient.getNRows() * (ambient.getNCols() + 1));
    for(const auto& rowString : rowStrings){
        gridString += rowString + '\n';
    }

    return gridString;
}

std::string Status::getTargetSnapshot(CompressedCoord start, CompressedCoord end, CompressedCoord actual) const{
    auto dm = ambient.getDistanceMatrix();
    auto rowStrings = ambient.getRowsStrings();

    auto fillCharCell = [&] (CompressedCoord cc, char fillWithChar){
        auto coord = dm.from1Dto2D(cc);
        assert(ambient.isValid(coord));
        rowStrings[coord.row][coord.col] = fillWithChar;
    };

    fillCharCell(start, 's');
    fillCharCell(actual, 'a');
    fillCharCell(end, 'e');

    std::string gridString;
    gridString.reserve(ambient.getNRows() * (ambient.getNCols() + 1));
    for(const auto& rowString : rowStrings){
        gridString += rowString + '\n';
    }

    return gridString;
}

const AmbientMap& Status::getAmbient() const{
    return ambient;
}

bool Status::dockingConflict(TimeStep sinceT, CompressedCoord pos, int agentId) const {
    if(noConflicts){
        return false;
    }

    auto predicate = [sinceT, pos](const PathWrapper& pW){
        const auto& path = pW.getPath();

        // I am ahead of other path
        if(sinceT >= path.size()){
            return false;
        }

        return std::any_of(path.cbegin() + sinceT, path.cend(), [pos](CompressedCoord cc){return cc == pos;});
    };

    return std::any_of(
        pathsWrappers.cbegin(),
        pathsWrappers.cbegin() + agentId,
        predicate
    ) ||
    std::any_of(
        pathsWrappers.cbegin() + agentId + 1,
        pathsWrappers.cend(),
        predicate
    );
}

bool Status::isDocking(int agentId, TimeStep t) const {
    TimeStep dockingTimeStep = std::ssize(pathsWrappers.getPath(agentId)) - 1;
    return t >= dockingTimeStep;
}

std::unordered_set<int> Status::getAvailableTasksIds() const {
    std::unordered_set<int> result;
    std::ranges::transform(notAssignedTasks | std::views::keys, std::inserter(result, result.end()), [](int key) { return key; });
    return result;
}

bool Status::noMoreTasks(int nextTasksIndex) const {
    return nextTasksIndex == notAssignedTasks.size();
}

bool Status::allTasksSatisfied() const {

    // 0 means task not satysfied
    // > 1 means tasks satysfied by more pWs (impossible)
    auto rightTaskCount = [this](const std::pair<int, Task>& taskItem){
        return std::count_if(
            pathsWrappers.cbegin(),
            pathsWrappers.cend(),
            [&taskItem](const PathWrapper& pW){return pW.getSatisfiedTasksIds().contains(taskItem.first);}
        ) == 1;
    };


    return std::ranges::all_of(notAssignedTasks, rightTaskCount);
}

void Status::updateTasks(std::unordered_map<int, Task> newTasks) {
    notAssignedTasks.merge(newTasks);
}

bool Status::taskIdExists(int taskId) const {
    return notAssignedTasks.contains(taskId) || assignedTasks.contains(taskId);
}

void Status::removeSatisfiedTasks(const std::unordered_set<int> &removedTasksIds) {
    for(int taskId : removedTasksIds){
        assert(assignedTasks.contains(taskId) && !notAssignedTasks.contains(taskId));
        Task removedTask = assignedTasks.find(taskId)->second;
        assert(removedTask.getIndex() == taskId);
        assignedTasks.erase(taskId);
        notAssignedTasks.emplace(taskId, removedTask);
    }
}
