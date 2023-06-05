
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "AmbientMap.hpp"
#include "DistanceMatrix.hpp"


std::vector<bool> AmbientMap::loadGrid(const std::filesystem::path &gridPath) {
    std::fstream fs(gridPath.c_str(), std::ios::in);

    if(!fs.is_open()){
        throw std::runtime_error("Grid file does not exist");
    }

    std::string line;

    using namespace boost::algorithm;

    std::vector<bool> grid{};

    while(std::getline(fs, line)){
        trim(line);

        std::ranges::transform(
            line,
            std::back_inserter(grid),
            [](char c){return c == static_cast<char>(CellType::OBSTACLE);}
        );
    }

    if(grid.empty()){
        throw std::runtime_error("Grid file is empty");
    }

    grid.shrink_to_fit();
    return grid;
}

bool AmbientMap::isValid(const Coord &coord) const {
    bool isInsideGrid = coord.row < getNRows() && coord.col < getNCols() &&
        coord.row >= 0 && coord.col >= 0;

    return isInsideGrid && !operator[](distanceMatrix.from2Dto1D(coord));
}

bool AmbientMap::operator[](CompressedCoord cc) const {
    return grid[cc];
}

int AmbientMap::getNRows() const {
    return distanceMatrix.nRows;
}

int AmbientMap::getNCols() const {
    return distanceMatrix.nCols;
}

std::optional<CompressedCoord> AmbientMap::movement(CompressedCoord coord, int directionIndex) const{
    assert(directionIndex >= 0 && directionIndex < directionVector.size());
    auto neighbor = distanceMatrix.from1Dto2D(coord) + directionVector[directionIndex];

    return isValid(neighbor) ? std::optional{distanceMatrix.from2Dto1D(neighbor)} : std::nullopt;
}

AmbientMap::AmbientMap(const std::filesystem::path &gridPath, const std::filesystem::path &distanceMatrixPath) :
    distanceMatrix{distanceMatrixPath},
    grid{loadGrid(gridPath)}
{}

const DistanceMatrix& AmbientMap::getDistanceMatrix() const{
    return distanceMatrix;
}

const std::vector<bool>& AmbientMap::getGrid() const {
    return grid;
}

std::vector<std::string> AmbientMap::getRowsStrings() const {
    std::vector<std::string> rowStrings;
    rowStrings.reserve(getNRows());

    for (int iRow = 0 ; iRow < getNRows() ; ++iRow){
        std::string rowString;
        rowString.reserve(getNCols());

        auto from = grid.cbegin() + iRow * getNCols();
        auto to = from + getNCols();

        std::ranges::transform(
            from,
            to,
            std::back_inserter(rowString),
            [](bool cell){
                auto isObstacle = cell ? CellType::OBSTACLE : CellType::FLOOR;
                return static_cast<char>(isObstacle);
            }
        );
        rowStrings.push_back(std::move(rowString));
    }

    return rowStrings;
}

std::vector<CompressedCoord> AmbientMap::getNeighbors(CompressedCoord actualLoc) const {
    std::vector<CompressedCoord> neighbors{};

    for(int i = 0 ; i < nDirections ; ++i){
        auto n = movement(actualLoc, i);
        if(n.has_value()){
            neighbors.push_back(*n);
        }
    }

    return neighbors;
}
