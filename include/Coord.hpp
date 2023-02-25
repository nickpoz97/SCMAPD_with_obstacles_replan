//
// Created by nicco on 02/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_COORD_HPP
#define SIMULTANEOUS_CMAPD_COORD_HPP


#include <vector>
#include <string>
#include "NewTypes.hpp"

struct Coord {
    int row;
    int col;

    using Direction = Coord;
    friend Coord operator+(const Coord& coord, const Direction& movement);
    friend bool operator==(const Coord& a, const Coord& b) = default;

};
using Direction = Coord::Direction;

using CompressedCoord = int;

using Path = std::vector<CompressedCoord>;

class VerbosePath : public std::vector<Coord>{
public:
    explicit operator std::string() const;
};

#endif //SIMULTANEOUS_CMAPD_COORD_HPP
