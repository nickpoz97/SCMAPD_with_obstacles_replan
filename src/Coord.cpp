//
// Created by nicco on 02/01/2023.
//

#include "Coord.hpp"
#include "fmt/printf.h"

Coord operator+(const Coord& coord, const Direction& movement){
    return {coord.row + movement.row, coord.col + movement.col};
}

Coord::operator std::string() const {
    return fmt::sprintf("{%d,%d}", row, col);
}
