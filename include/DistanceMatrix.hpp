//
// Created by nicco on 30/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP
#define SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP

#include <filesystem>
#include <cnpy.h>
#include "TypeDefs.hpp"
#include "custom_types.h"

struct DistanceMatrix {
    explicit DistanceMatrix(cnpy::NpyArray&& data);
    [[nodiscard]] int getDistance(CompressedCoord from, CompressedCoord to) const;
    [[nodiscard]] int getDistance(Coord from, Coord to) const;

    static CompressedCoord from2Dto1D(int col, int row, size_t nCols);
    static CompressedCoord from2Dto1D(cmapd::Point point, size_t nCols);

    [[nodiscard]] int computeCumulatedValue(cmapd::Point x,
                                   int label,
                                   const cmapd::path_t& goal_sequence) const;

    const cnpy::NpyArray rawDistanceMatrix;
    const int nRows;
    const int nCols;

    const int startCoordsSize;
    const int endCoordsSize;
};

#endif //SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP
