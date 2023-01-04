//
// Created by nicco on 04/01/2023.
//

#include "MAPF/ExploredSet.hpp"

void ExploredSet::add(const Node& node){
    exploredSet[node.getLocation()].insert(node.getGScore());
}

bool ExploredSet::contains(CompressedCoord loc, TimeStep t) const{
    return exploredSet.contains(loc) && exploredSet.at(loc).contains(t);
}

void ExploredSet::clear() {
    exploredSet.clear();
}
