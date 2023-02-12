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
    explicit ExploredSet(int maxPosVisits);

    void insert(const Node& node);

    bool contains(const Node& node) const;
    bool contains(CompressedCoord loc, TimeStep t) const;

    void clear();
private:
    int maxPosVisits;
    std::unordered_map<CompressedCoord, std::unordered_set<TimeStep>> exploredNodesSet{};
    std::unordered_map<CompressedCoord, int> exploredPosCounter;
};


#endif //SIMULTANEOUS_CMAPD_EXPLOREDSET_HPP
