//
// Created by nicco on 16/02/2023.
//

#include "MAPF/LimitedExploredSet.hpp"

LimitedExploredSet::LimitedExploredSet(int maxPosVisits) :
    maxPosVisits{maxPosVisits}
    {}

void LimitedExploredSet::insert(const Node &node) {
    ExploredSet::insert(node);
    ++exploredPosCounter[node.getLocation()];
}

bool LimitedExploredSet::contains(const Node &node) const {
    return contains(node.getLocation(), node.getGScore());
}

bool LimitedExploredSet::contains(CompressedCoord loc, TimeStep t) const {
    return ExploredSet::contains(loc, t) ||
            (exploredPosCounter.contains(loc) && exploredPosCounter.at(loc) >= maxPosVisits);
}

void LimitedExploredSet::clear() {
    ExploredSet::clear();
    exploredPosCounter.clear();
}
