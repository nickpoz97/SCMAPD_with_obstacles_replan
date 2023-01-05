//
// Created by nicco on 05/12/2022.
//

#include <boost/iterator/counting_iterator.hpp>
#include <fmt/core.h>
#include "Status.hpp"
#include "fmt/color.h"

Status::Status(AmbientMap &&ambientMap, int nRobots,
               std::vector<Task> &&tasks) :
        ambient(std::move(ambientMap)),
        tasksVector(std::move(tasks)),
        paths(nRobots),
        unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasksVector.size()))
    {}

const Task & Status::getTask(int i) const {
    return tasksVector[i];
}

const std::unordered_set<int> &Status::getUnassignedTasksIndices() const{
    return unassignedTasksIndices;
}

const std::vector<Task> &Status::getTasks() const {
    return tasksVector;
}

void Status::removeTaskIndex(int i) {
    unassignedTasksIndices.erase(i);
}

void Status::update(Path &&path, int agentId) {
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
    assert(agentId > 0 && agentId < paths.size());

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

int Status::getDistance(const Coord& from, const Coord& to) const {
    return ambient.getDistance(from, to);
}

int Status::getDistance(CompressedCoord from, CompressedCoord to) const {
    return ambient.getDistance(from, to);
}

CompressedCoord Status::toCompressedCoord(const Coord &c) const {
    return ambient.toCompressedCoord(c);
}

Coord Status::toCoord(CompressedCoord c) const{
    return ambient.toCoord(c);
}

bool Status::checkAllConflicts(bool printConflicts) const {
    bool conflictFound = false;

    for(int i = 0 ; i < paths.size() ; ++i){
        for(int j = i+1 ; j < paths.size() ; ++j){
            const auto& pA = paths[i];
            const auto& pB = paths[j];

            if(pA.empty() || pB.empty()){
                continue;
            }
            for(int t = 0 ; t < std::max(paths[i].size(), paths[j].size()) ; ++t){
                auto tA = std::min(t, static_cast<int>(pA.size()-1));
                auto tB = std::min(t, static_cast<int>(pB.size()-1));

                bool nodeConflict = pA[std::min(t, static_cast<int>(pA.size()-1))] == pB[std::min(t, static_cast<int>(pB.size()-1))];
                bool edgeConflict = t < pA.size()-1 && t < pB.size() && pA[t] == pB[t+1] && pA[t+1] == pB[t];

                if(printConflicts && nodeConflict){
                    auto posString = static_cast<std::string>(toCoord(pA[std::min(t, static_cast<int>(pA.size()-1))]));
                    fmt::print("Node conflict between agents {}-{} at t = {} in pos = {}", i,j,t, posString);
                }

                if(printConflicts && edgeConflict){
                    auto pos1String = static_cast<std::string>(toCoord(pA[t]));
                    auto pos2String = static_cast<std::string>(toCoord(pA[t+1]));

                    fmt::print("Edge conflict between agents {}-{} at t = {} and {} in pos = {} and {}",
                        i,j,t,t+1, pos1String, pos2String);
                }

                conflictFound = conflictFound || nodeConflict || edgeConflict;
            }
        }
    }
    return conflictFound;
}

