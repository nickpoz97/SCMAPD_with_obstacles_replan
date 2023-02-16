//
// Created by nicco on 16/02/2023.
//

#ifndef SIMULTANEOUS_CMAPD_LIMITEDEXPLOREDSET_HPP
#define SIMULTANEOUS_CMAPD_LIMITEDEXPLOREDSET_HPP


#include "ExploredSet.hpp"

class LimitedExploredSet : public ExploredSet{
public:
    explicit LimitedExploredSet(int maxPosVisits);

    void insert(const Node &node) override;

    bool contains(const Node &node) const override;

    bool contains(CompressedCoord loc, TimeStep t) const override;

    void clear() override;

private:
    const int maxPosVisits;
    std::unordered_map<CompressedCoord, int> exploredPosCounter;
};


#endif //SIMULTANEOUS_CMAPD_LIMITEDEXPLOREDSET_HPP
