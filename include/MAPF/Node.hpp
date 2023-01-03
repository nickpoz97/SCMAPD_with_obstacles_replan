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
    Node(CompressedCoord loc, TimeStep t, int hScore, const Node *parent = nullptr);

    [[nodiscard]] int getFScore() const;

    bool operator==(const Node& other) const;

    void update(Node const *newFather, int newG);

    [[nodiscard]] Path getPath(const Status &status) const;

    friend bool operator>(const Node& a, const Node& b);

    CompressedCoord getLocation() const;
private:
    const Node* father;
    const CompressedCoord location;
    const TimeStep time;

    int g;
    int h;
};


#endif //SIMULTANEOUS_CMAPD_NODE_HPP
