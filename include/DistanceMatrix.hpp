//
// Created by nicco on 30/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP
#define SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP

#include <filesystem>
#include <cnpy.h>
#include "TypeDefs.hpp"
#include "Coord.hpp"

struct DistanceMatrix {
    explicit DistanceMatrix(cnpy::NpyArray&& data);
    [[nodiscard]] int getDistance(CompressedCoord from, CompressedCoord to) const;
    [[nodiscard]] int getDistance(Coord from, Coord to) const;

    [[nodiscard]] CompressedCoord from2Dto1D(int col, int row) const;
    [[nodiscard]] CompressedCoord from2Dto1D(const Coord &point) const;

    [[nodiscard]] Coord from1Dto2D(CompressedCoord point) const;

    const cnpy::NpyArray rawDistanceMatrix;
    const int nRows;
    const int nCols;

    const int startCoordsSize;
    const int endCoordsSize;
};

#endif //SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP
