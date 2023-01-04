//
// Created by nicco on 03/01/2023.
//

#include <forward_list>
#include <utility>
#include "MAPF/Node.hpp"

bool Node::operator==(const Node &other) const{
    return location == other.location && g == other.g;
}

Node::Node(CompressedCoord loc, TimeStep t, int hScore, std::shared_ptr<Node> parentPtr) :
        father{std::move(parentPtr)},
        location{loc},
        g{t},
        h{hScore}
    {}

int Node::getFScore() const {
    return g+h;
}

std::forward_list<CompressedCoord> Node::getPathList() const {
    std::forward_list<CompressedCoord> pathList{};

    for(const Node* actualNode = this ; actualNode != nullptr ; actualNode = actualNode->father.get()){
        pathList.push_front(actualNode->location);
    }

    return pathList;
}

bool operator<(const Node &a, const Node &b) {
    return a.getFScore() < b.getFScore();
}

CompressedCoord Node::getLocation() const {
    return location;
}

TimeStep Node::getGScore() const {
    return g;
}
