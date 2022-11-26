//
// Created by nicco on 26/11/2022.
//

#include "PartialAssignment.hpp"
#include <limits>
#include <numeric>
#include "Assignment.hpp"

Assignment::Assignment(CompressedCoord startPosition, unsigned index, unsigned capacity) :
        startPosition{startPosition},
        index{index},
        capacity{capacity}
    {}

unsigned int Assignment::getCapacity() const {
    return capacity;
}

TimeStep Assignment::getTtd() const {
    return ttd;
}

unsigned Assignment::getIndex() const {
    return index;
}

CompressedCoord Assignment::getStartPosition() const {
    return startPosition;
}