//
// Created by nicco on 05/12/2022.
//

#include <fmt/core.h>
#include "Status.hpp"
#include "fmt/color.h"

Status::Status(AmbientMap &&ambientMap, int nRobots,
               std::vector<Task> &&tasks) :
        ambient(std::move(ambientMap)),
        tasksVector(std::move(tasks)),
        paths(nRobots)
        {}

const Task & Status::getTask(int i) const {
    return tasksVector[i];
}

const std::vector<Task> &Status::getTasks() const {
    return tasksVector;
}

void Status::updatePaths(Path &&path, int agentId) {
    longestPathSize = std::max(static_cast<int>(path.size()), longestPathSize);
    paths[agentId] = std::move(path);
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
    assert(agentId >= 0 && agentId < paths.size());

    auto predicate = [t1, coord1, coord2](const Path& p){
        // if path is empty there are no conflicts
        if(p.empty()){
            return false;
        }
        auto t2 = std::min(t1 + 1, static_cast<int>(p.size()-1));

        // todo check this
        bool nodeConflict = coord2 == p[t2];
        bool edgeConflict = t1 < (p.size() - 1) && coord1 == p[t2] && coord2 == p[t1];

        return nodeConflict || edgeConflict;
    };

    return std::ranges::any_of(paths.begin(), paths.begin() + agentId, predicate) ||
        std::ranges::any_of(paths.begin() + agentId + 1, paths.end(), predicate);
}

const std::vector<Path>& Status::getPaths() const {
    return paths;
}

const DistanceMatrix& Status::getDistanceMatrix() const{
    return ambient.getDistanceMatrix();
}

bool Status::checkPathWithStatus(const Path &path, int agentId) const{
    return std::ranges::any_of(
        paths.begin(),
        paths.begin() + agentId,
        [&](const Path& other){return checkPathConflicts(path, other);}
    ) ||
    std::ranges::any_of(
        paths.begin() + agentId + 1,
        paths.end(),
        [&](const Path& other){return checkPathConflicts(path, other);}
    );
}

bool Status::checkAllConflicts() const {
    for(int i = 0 ; i < paths.size() ; ++i){
        for(int j = i+1 ; j < paths.size() ; ++j){
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

    return checkPathConflicts(paths[i], paths[j]);
}

bool Status::checkPathConflicts(const Path &pA, const Path &pB) {
    if(pA.empty() || pB.empty()){
        return false;
    }

    for(int t = 0 ; t < std::max(pA.size(), pB.size()) ; ++t) {
        bool nodeConflict =
                pA[std::min(t, static_cast<int>(pA.size() - 1))] == pB[std::min(t, static_cast<int>(pB.size() - 1))];
        bool edgeConflict = t < pA.size() - 1 && t < pB.size() && pA[t] == pB[t + 1] && pA[t + 1] == pB[t];

        if(nodeConflict || edgeConflict){
            return true;
        }
    }
    return false;
}

TimeStep Status::getLongestPathSize() const {
    return longestPathSize;
}

