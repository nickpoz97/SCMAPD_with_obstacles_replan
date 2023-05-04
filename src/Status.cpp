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
    assert(tasks.contains(i));
    return tasks.find(i)->second;
}

std::pair<int, int> Status::update(ExtractedPath extractedPath) {
    auto agentId = extractedPath.agentId;
    auto taskId = extractedPath.newTaskId;

    assert(!assignedTasks.contains(taskId));
    pathsWrappers[agentId] = std::move(extractedPath.wrapper);

    return {agentId, taskId};
}

std::vector<CompressedCoord>
Status::getValidNeighbors(int agentId, CompressedCoord c, TimeStep t) const {
    std::vector<CompressedCoord> neighbors;
    neighbors.reserve(AmbientMap::nDirections);

    for(int i = 0 ; i < AmbientMap::nDirections ; ++i){
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
    assert(agentId >= 0 && agentId < getPathWrappers().size());

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

const PWsVector & Status::getPathWrappers() const {
    return pathsWrappers;
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

std::vector<int> Status::chooseNWorstTasks(int n, Metric mt, const std::vector<int> &coveredTasks) const {
    // taskId, value
    using TaskInfo = std::pair<int, TimeStep>;

    n = std::min(static_cast<int>(coveredTasks.size()), n);

    std::vector<TaskInfo> orderedTasks;
    orderedTasks.reserve(coveredTasks.size());

    // get taskId and relative metric (delay or arrival time)
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

    // sort taskIds using the metric
    std::ranges::sort(
        orderedTasks,
        [](const TaskInfo& ta, const TaskInfo& tb){return ta.second > tb.second;}
    );

    std::vector<int> taskIndicesToRemove{};
    taskIndicesToRemove.reserve(n);

    for(int i = 0 ; i < n ; ++i){
        taskIndicesToRemove.push_back(orderedTasks[i].first);
    }
    return taskIndicesToRemove;
}

std::vector<int> Status::chooseNRandomTasks(int iterIndex, int n, const std::vector<int> &coveredTasks) const{
    n = std::min(static_cast<int>(coveredTasks.size()), n);

    std::vector<int> shuffled_tasks{};
    shuffled_tasks.reserve(coveredTasks.size());

    // copy indices
    std::ranges::transform(
        coveredTasks,
        std::back_inserter(shuffled_tasks),
        [](int taskId){return taskId;}
    );

    auto seed = hash_value(*this);
    boost::hash_combine(seed, iterIndex);

    // shuffle them using status hash as seed
    std::shuffle(
        shuffled_tasks.begin(),
        shuffled_tasks.end(),
        std::default_random_engine(seed)
    );
    return {shuffled_tasks.begin(), shuffled_tasks.begin() + n};
}

std::vector<int> Status::chooseTasksFromNWorstAgents(int iterIndex, int n, Metric mt) const {
    // agentId, value
    using AgentInfo = std::pair<int, TimeStep>;

    int nAvailableAgents = std::ssize(getPathWrappers());
    n = std::min(nAvailableAgents, n);

    std::vector<AgentInfo> orderedAgents;
    orderedAgents.reserve(orderedAgents.size());

    for (int i = 0 ; i < nAvailableAgents ; ++i){
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

    std::vector<int> taskIndicesToRemove{};

    for(int i = 0 ; i < n ; ++i){
        const auto& pW = pathsWrappers[orderedAgents[i].first];
        if(!pW.getSatisfiedTasksIds().empty()) {
            taskIndicesToRemove.push_back(pW.randomTaskId(iterIndex));
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
        pathsWrappers.cend(),
        predicate
    );
}

bool Status::isDocking(int agentId, TimeStep t) const {
    TimeStep dockingTimeStep = std::ssize(pathsWrappers.getPath(agentId)) - 1;
    return t >= dockingTimeStep;
}

void Status::setTasks(const std::vector<Task>& newTasks) {
    for(const auto& task : newTasks){
        tasks.emplace(task.getIndex(), task);
    }
}

bool Status::taskIdExists(int taskId) const {
    return tasks.contains(taskId);
}

std::vector<int> Status::getTasksIds() const {
    auto ids = tasks | std::views::keys;

    return {ids.begin(), ids.end()};
}

