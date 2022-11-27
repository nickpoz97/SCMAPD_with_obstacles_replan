//
// Created by nicco on 26/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP


#include "Task.hpp"
#include "PartialAssignment.hpp"
#include <list>
#include "TypeDefs.hpp"

class Assignment {

public:
    explicit Assignment(CompressedCoord startPosition, unsigned index, unsigned capacity);

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] TimeStep getTtd() const;

    [[nodiscard]] unsigned getIndex() const;

    [[nodiscard]] CompressedCoord getStartPosition() const;

    [[nodiscard]] const Path &getPath() const;

    friend bool operator<(const Assignment& a, const Assignment& b);

    void update(Assignment&& assignment);

protected:
    CompressedCoord startPosition;
    unsigned capacity;
    unsigned index;
    Path path{};
    TimeStep ttd = 0;
};


#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
