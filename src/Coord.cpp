//
// Created by nicco on 02/01/2023.
//

#include "Coord.hpp"

Coord operator+(const Coord& coord, const Movement& movement){
    return {coord.row + movement.row, coord.col + movement.col};
}
