//
// Created by nicco on 02/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_COORD_HPP
#define SIMULTANEOUS_CMAPD_COORD_HPP


#include <vector>

struct Coord {
    int row;
    int col;
};
using CompressedCoord = int;

using Movement = Coord;

Coord operator+(const Coord& coord, const Movement& movement);

using Path = std::vector<Coord>;
using CompressedPath = std::vector<CompressedCoord>;


#endif //SIMULTANEOUS_CMAPD_COORD_HPP
