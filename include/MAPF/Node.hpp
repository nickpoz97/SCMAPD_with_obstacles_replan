//
// Created by nicco on 03/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_NODE_HPP
#define SIMULTANEOUS_CMAPD_NODE_HPP

#include "Coord.hpp"
#include "TypeDefs.hpp"
#include "Status.hpp"

class Node {
public:
    Node(CompressedCoord loc, TimeStep t, int hScore, std::shared_ptr<Node> parentPtr = nullptr);

    [[nodiscard]] int getFScore() const;

    bool operator==(const Node& other) const;

    [[nodiscard]] std::list<CompressedCoord> getPathList() const;

    friend bool operator<(const Node& a, const Node& b);

    [[nodiscard]] CompressedCoord getLocation() const;

    [[nodiscard]] TimeStep getGScore() const;

private:
    const std::shared_ptr<Node> father;
    const CompressedCoord location;

    const int g;
    const int h;
};


#endif //SIMULTANEOUS_CMAPD_NODE_HPP
