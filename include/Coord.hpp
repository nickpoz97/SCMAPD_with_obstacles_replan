//
// Created by nicco on 02/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_COORD_HPP
#define SIMULTANEOUS_CMAPD_COORD_HPP


#include <vector>
#include <string>

struct Coord {
    int row;
    int col;

    using Direction = Coord;
    friend Coord operator+(const Coord& coord, const Direction& movement);
    friend bool operator==(const Coord& a, const Coord& b) = default;

    explicit operator std::string() const;
};
using Direction = Coord::Direction;

using CompressedCoord = int;

using Path = std::vector<CompressedCoord>;
using CompressedPath = std::vector<CompressedCoord>;


#endif //SIMULTANEOUS_CMAPD_COORD_HPP
