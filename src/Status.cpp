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

    // todo chech edge collisions
    for(int i = 0 ; i < AmbientMap::nDirections ; ++i){
        auto result = ambient.movement(c, i);
        if(result.has_value() && !occupiedByOtherAgent(agentId, result.value(), t + 1)){
            neighbors.push_back(result.value());
        }
    }

    return neighbors;
}

bool Status::occupiedByOtherAgent(int agentId, CompressedCoord coord, TimeStep t) const{
    assert(agentId > 0 && agentId < paths.size());

    auto predicate = [t, &coord](const Path& p){
        // if path is empty than there are no conflicts
        return !p.empty() && coord == p[std::min(t+1, static_cast<int>(p.size()-1))];
    };

    // skipping path of actual agent
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

