//
// Created by nicco on 04/01/2023.
//

#include "MAPF/ExploredSet.hpp"

void ExploredSet::insert(const Node& node){
    assert(!contains(node));
    exploredNodesSet[node.getLocation()].emplace(node.getGScore());
}

bool ExploredSet::contains(CompressedCoord loc, TimeStep t) const{
    return exploredNodesSet.contains(loc) && exploredNodesSet.at(loc).contains(t);
}

bool ExploredSet::contains(const Node& node) const{
    return contains(node.getLocation(), node.getGScore());
}

void ExploredSet::clear() {
    exploredNodesSet.clear();
}
