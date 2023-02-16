//
// Created by nicco on 04/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_EXPLOREDSET_HPP
#define SIMULTANEOUS_CMAPD_EXPLOREDSET_HPP

#include <unordered_map>
#include <unordered_set>
#include "Coord.hpp"
#include "NewTypes.hpp"
#include "Node.hpp"

class ExploredSet {
public:
    explicit ExploredSet() = default;

    virtual void insert(const Node& node);

    virtual bool contains(const Node& node) const;
    virtual bool contains(CompressedCoord loc, TimeStep t) const;

    virtual void clear();
protected:
    std::unordered_map<CompressedCoord, std::unordered_set<TimeStep>> exploredNodesSet;
};


#endif //SIMULTANEOUS_CMAPD_EXPLOREDSET_HPP
