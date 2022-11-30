//
// Created by nicco on 30/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP
#define SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP

#include "Assignment.hpp"
#include <SCMAPD.hpp>
#include <filesystem>
#include <cnpy.h>
#include "TypeDefs.hpp"
#include "custom_types.h"

class DistanceMatrix {
public:
    DistanceMatrix(cnpy::NpyArray&& data, unsigned nCols);
    [[nodiscard]] unsigned getDistance(CompressedCoord from, CompressedCoord to) const;
    [[nodiscard]] unsigned getDistance(cmapd::Point from, cmapd::Point to) const;

    static CompressedCoord from2Dto1D(unsigned int col, unsigned int row, size_t nCols);
    static CompressedCoord from2Dto2D(cmapd::Point point, size_t nCols);

    [[nodiscard]] unsigned computeCumulatedValue(cmapd::Point x,
                                   int label,
                                   const cmapd::path_t& goal_sequence) const;
private:
    const cnpy::NpyArray rawDistanceMatrix;
    const unsigned nCols;

    const unsigned startCoordsSize;
    const unsigned endCoordsSize;
};

#endif //SIMULTANEOUS_CMAPD_DISTANCEMATRIX_HPP
