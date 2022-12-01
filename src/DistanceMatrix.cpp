#include "Assignment.hpp"
#include <fstream>
#include <cnpy.h>
#include "DistanceMatrix.hpp"

DistanceMatrix::DistanceMatrix(cnpy::NpyArray &&data, unsigned int nCols) :
    rawDistanceMatrix{data},
    nCols{nCols},
    startCoordsSize{static_cast<unsigned int>(rawDistanceMatrix.shape[0] * rawDistanceMatrix.shape[1])},
    endCoordsSize{static_cast<unsigned int>(rawDistanceMatrix.shape[2] * rawDistanceMatrix.shape[3])}
    {}

unsigned DistanceMatrix::getDistance(Coord from, Coord to) const {
    return getDistance(
            from2Dto1D(from.col, from.row, nCols), from2Dto1D(to.col, to.row, nCols)
    );
}

CompressedCoord DistanceMatrix::from2Dto1D(unsigned col, unsigned row, size_t nCols) {
    return row * static_cast<unsigned>(nCols) + col;
}

unsigned DistanceMatrix::getDistance(CompressedCoord from, CompressedCoord to) const {
    // distance matrix is considered double
    const auto* ptr = rawDistanceMatrix.data<double>();
    return static_cast<unsigned>(ptr[from * startCoordsSize + to]);
}

unsigned
DistanceMatrix::computeCumulatedValue(cmapd::Point x, int label, const cmapd::path_t &goal_sequence) const {
    auto h_value{getDistance(x, goal_sequence[label])};
    for (int j{label + 1}; j < goal_sequence.size(); ++j) {
        h_value += getDistance(goal_sequence[j - 1], goal_sequence[j]);
    }
    return h_value;
}

CompressedCoord DistanceMatrix::from2Dto2D(cmapd::Point point, size_t nCols) {
    return point.row * nCols + point.col;
}
