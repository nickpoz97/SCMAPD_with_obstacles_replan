//
// Created by nicco on 03/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_NODE_HPP
#define SIMULTANEOUS_CMAPD_NODE_HPP

#include "Coord.hpp"
#include "NewTypes.hpp"
#include "MAPD/Status.hpp"

class Node {
public:
    Node(CompressedCoord loc, TimeStep t, const DistanceMatrix& dm, const std::pair<int, CompressedCoord>& goal, const Node* parentPtr = nullptr);

    [[nodiscard]] int getFScore() const;

    bool operator==(const Node& other) const;

    [[nodiscard]] std::list<CompressedCoord> getPathList() const;

    friend int operator>(const Node& a, const Node& b);

    [[nodiscard]] CompressedCoord getLocation() const;

    [[nodiscard]] TimeStep getGScore() const;

    [[nodiscard]] int getNextTargetIndex(int lastIndex) const;

    [[nodiscard]] CompressedCoord getTargetPosition() const;

    [[nodiscard]] int getTargetIndex() const;
private:
    const Node* father;
    CompressedCoord location;
    CompressedCoord targetPosition;
    int targetIndex;

    [[nodiscard]] bool targetReached() const;

    int g;
    int h;
};


#endif //SIMULTANEOUS_CMAPD_NODE_HPP
