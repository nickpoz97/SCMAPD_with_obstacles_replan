#include <fstream>
#include <cnpy.h>
#include "DistanceMatrix.hpp"
#include "Coord.hpp"

DistanceMatrix::DistanceMatrix(const std::filesystem::path& data) :
    rawDistanceMatrix{cnpy::npy_load(data.string())},
    nRows{static_cast<int>(rawDistanceMatrix.shape[0])},
    nCols{static_cast<int>(rawDistanceMatrix.shape[1])},
    startCoordsSize{static_cast<int>(rawDistanceMatrix.shape[0] * rawDistanceMatrix.shape[1])},
    endCoordsSize{static_cast<int>(rawDistanceMatrix.shape[2] * rawDistanceMatrix.shape[3])}
    {
        if(startCoordsSize != endCoordsSize || rawDistanceMatrix.shape.size() != 4){
            throw std::runtime_error("Loaded wrong distance matrix");
        }
    }

int DistanceMatrix::getDistance(const Coord &from, const Coord &to) const {
    return getDistance(from2Dto1D(from), from2Dto1D(to));
}

CompressedCoord DistanceMatrix::from2Dto1D(int row, int col) const{
    return row * static_cast<int>(nCols) + col;
}

int DistanceMatrix::getDistance(CompressedCoord from, CompressedCoord to) const {
    // distance matrix is considered double
    const auto* ptr = rawDistanceMatrix.data<double>();
    return static_cast<int>(ptr[from * endCoordsSize + to]);
}

CompressedCoord DistanceMatrix::from2Dto1D(const Coord &point) const{
    return from2Dto1D(point.row, point.col);
}

Coord DistanceMatrix::from1Dto2D(CompressedCoord point) const {
    return {point / nCols, point % nCols};
}
