//
// Created by nicco on 04/01/2023.
//

#include "MAPF/ExploredSet.hpp"

void ExploredSet::insert(const Node& node){
    auto location = node.getLocation();

    assert(!contains(node));

    exploredNodesSet[location].emplace(node.getGScore());
    ++exploredPosCounter[location];
}

bool ExploredSet::contains(CompressedCoord loc, TimeStep t) const{
    return
        (exploredPosCounter.contains(loc) && exploredPosCounter.at(loc) >= maxPosVisits) ||
        (exploredNodesSet.contains(loc) && exploredNodesSet.at(loc).contains(t));
}

bool ExploredSet::contains(const Node& node) const{
    return contains(node.getLocation(), node.getGScore());
}

void ExploredSet::clear() {
    exploredNodesSet.clear();
    exploredPosCounter.clear();
}

ExploredSet::ExploredSet(int maxPosVisits) : maxPosVisits{maxPosVisits} {}
