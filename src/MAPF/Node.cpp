//
// Created by nicco on 03/01/2023.
//

#include <list>
#include <utility>
#include "MAPF/Node.hpp"

bool Node::operator==(const Node &other) const{
    return location == other.location && g == other.g && targetIndex == other.targetIndex;
}

Node::Node(CompressedCoord loc, TimeStep t, const DistanceMatrix& dm, const std::pair<int, CompressedCoord>& goal, const Node* parentPtr) :
        father{parentPtr},
        location{loc},
        targetPosition{goal.second},
        targetIndex{goal.first},
        g{t},
        h{dm.getDistance(loc, goal.second)}
    {}

int Node::getFScore() const {
    return g+h;
}

std::list<CompressedCoord> Node::getPathList() const {
    std::list<CompressedCoord> pathList{};

    for(const Node* actualNode = this ; actualNode != nullptr ; actualNode = actualNode->father){
        pathList.push_front(actualNode->location);
    }

    return pathList;
}

CompressedCoord Node::getLocation() const {
    return location;
}

TimeStep Node::getGScore() const {
    return g;
}

int operator>(const Node &a, const Node &b) {
    return a.getTargetIndex() < b.getTargetIndex() ||
        (a.getTargetIndex() == b.getTargetIndex() && a.getFScore() > b.getFScore());
}

bool Node::targetReached() const {
    return location == targetPosition;
}

int Node::getNextTargetIndex() const {
    return targetReached() ? targetIndex + 1 : targetIndex;
}

CompressedCoord Node::getTargetPosition() const {
    return targetPosition;
}

int Node::getTargetIndex() const {
    return targetIndex;
}
