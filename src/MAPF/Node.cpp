//
// Created by nicco on 03/01/2023.
//

#include <forward_list>
#include "MAPF/Node.hpp"

bool Node::operator==(const Node &other) const{
    return location == other.location && time == other.time;
}

Node::Node(CompressedCoord loc, TimeStep t, int hScore, const Node *parent) :
    father{parent},
    location{loc},
    time{t},
    g{t},
    h{hScore}
    {}

int Node::getFScore() const {
    return g+h;
}

void Node::update(Node const *newFather, int newG) {
    newFather = newFather;
    g = newG;
}

Path Node::getPath(const Status &status) const {
    std::forward_list<CompressedCoord> pathList{};

    for(const Node* actualNode = this ; actualNode != nullptr ; actualNode = actualNode->father){
        pathList.push_front(actualNode->location);
    }

    return {std::make_move_iterator(pathList.begin()), std::make_move_iterator(pathList.end())};
}

bool operator>(const Node &a, const Node &b) {
    return a.getFScore() > b.getFScore();
}

CompressedCoord Node::getLocation() const {
    return location;
}
