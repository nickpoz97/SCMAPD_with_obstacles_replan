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
    paths[agentId] = std::move(path);
}

std::vector<CompressedCoord> Status::getValidNeighbors(int agentId, CompressedCoord c, TimeStep t) const {
    std::vector<CompressedCoord> neighbors;
    neighbors.reserve(AmbientMap::nDirections);

    for(int i = 0 ; i < AmbientMap::nDirections ; ++i){
        auto result = ambient.movement(c, i);
        if(result.has_value() && !checkDynamicObstacle(agentId, c, result.value(), t)){
            neighbors.push_back(result.value());
        }
    }

    return neighbors;
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

bool Status::checkAllConflicts(bool printConflicts) const {
    bool conflictFound = false;

    for(int i = 0 ; i < paths.size() ; ++i){
        for(int j = i+1 ; j < paths.size() ; ++j){
                conflictFound = checkPathConflicts(i, j, printConflicts);
        }
    }
    return conflictFound;
}

bool Status::checkPathConflicts(int i, int j, bool printConflicts) const{
    if(i == j){
        return false;
    }

    bool result = checkPathConflicts(paths[i], paths[j], printConflicts);
    if(result && printConflicts){
        fmt::print("Between agents {}, {}", i, j);
    }
    return result;
}

bool Status::checkPathConflicts(const Path & pA, const Path& pB, bool printConflicts) const {
    const auto& dm = getDistanceMatrix();

    if(pA.empty() || pB.empty()){
        return false;
    }

    for(int t = 0 ; t < std::max(pA.size(), pB.size()) ; ++t) {
        bool nodeConflict =
                pA[std::min(t, static_cast<int>(pA.size() - 1))] == pB[std::min(t, static_cast<int>(pB.size() - 1))];
        bool edgeConflict = t < pA.size() - 1 && t < pB.size() && pA[t] == pB[t + 1] && pA[t + 1] == pB[t];

        if(!printConflicts && (nodeConflict || edgeConflict)){
            return true;
        }

        if (nodeConflict) {
            auto posString = static_cast<std::string>(dm.from1Dto2D(pA[std::min(t, static_cast<int>(pA.size() - 1))]));
            fmt::print("Node conflict at t = {} in pos = {}", t, posString);
        }

        if (edgeConflict) {
            auto pos1String = static_cast<std::string>(dm.from1Dto2D(pA[t]));
            auto pos2String = static_cast<std::string>(dm.from1Dto2D(pA[t + 1]));

            fmt::print("Edge conflict at t = {} and {} in pos = {} and {}",
                       t, t + 1, pos1String, pos2String);
        }
    }
    return false;
}

