
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "AmbientMap.hpp"
#include "DistanceMatrix.hpp"


std::vector<std::vector<CellType>> AmbientMap::getGrid(const std::filesystem::path &gridPath) {
    std::fstream fs(gridPath.c_str(), std::ios::in);

    if(!fs.is_open()){
        throw std::runtime_error("Grid file doesn' t exist");
    }
    std::list<std::list<CellType>> tmpGrid;

    using namespace boost::algorithm;

    std::string line;
    while(std::getline(fs, line)){
        std::list<CellType> row;

        trim(line);

        auto charConverter = [](char c){
            if(c != static_cast<char>(CellType::ENDPOINT)){
                return static_cast<CellType>(c);
            }
            return CellType::FLOOR;
        };
        std::transform(line.cbegin(), line.cend(), std::back_inserter(row), charConverter);

        tmpGrid.push_back(row);
    }

    if(tmpGrid.empty()){
        throw std::runtime_error("Grid file is empty");
    }

    int nRows = tmpGrid.size();
    int nCols = tmpGrid.cbegin()->size();

    std::vector<std::vector<CellType>> grid{};
    grid.reserve(nRows);

    for(const auto& tmpRow : tmpGrid){
        std::vector<CellType> row{tmpRow.begin(), tmpRow.end()};
        grid.push_back(row);
    }

    return grid;
}

bool AmbientMap::isValid(const Coord &coord) const {
    bool isInsideGrid = coord.row < getNRows() && coord.col < getNCols() &&
        coord.row >= 0 && coord.col >= 0;

    return isInsideGrid && operator[](coord) != CellType::OBSTACLE;
}

CellType AmbientMap::operator[](const Coord &coord) const {
    return grid[coord.row][coord.col];
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
    grid{getGrid(gridPath)}
{
    if(distanceMatrix.nRows != grid.size() || (!grid.empty() && distanceMatrix.nCols != grid[0].size())){
        throw std::runtime_error("Grid file and distance matrix file do not refer to same ambient");
    }
}

const DistanceMatrix& AmbientMap::getDistanceMatrix() const{
    return distanceMatrix;
}
