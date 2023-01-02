
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "AmbientMap.hpp"
#include "DistanceMatrix.hpp"

std::fstream AmbientMap::openGridFile(const std::filesystem::path &gridPath){
    std::fstream fs(gridPath.c_str(), std::ios::in);

    if(!fs.is_open()){
        throw std::runtime_error("Grid file doesn' t exist");
    }

    return fs;
}


std::vector<std::vector<CellType>> AmbientMap::getGrid(std::fstream &&data) {
    std::string line;
    std::getline(data, line);

    if(line.empty()){
        throw std::runtime_error("Empty grid file");
    }

    char horizontalSep = ',';

    std::stringstream lineStream{line};
    std::string token;

    std::getline(lineStream, token, horizontalSep);
    int nRows = std::stoi(token);

    std::getline(lineStream, token, horizontalSep);
    int nCols = std::stoi(token);

    std::vector<std::vector<CellType>> grid;
    grid.reserve(nRows);

    using namespace boost::algorithm;
    for(int i = 0 ; i < nRows ; ++i){
        std::vector<CellType> row;
        row.reserve(nCols);

        std::getline(data, line);
        trim(line);

        auto charConverter = [](char c){
            if(c != static_cast<char>(CellType::ENDPOINT)){
                return static_cast<CellType>(c);
            }
            return CellType::FLOOR;
        };

        std::transform(line.cbegin(), line.cend(), std::back_inserter(row), charConverter);

        grid.push_back(row);
    }
}

bool AmbientMap::isValid(Coord coord) const {
    bool isInsideGrid = coord.row <= getNRows() && coord.col <= getNCols() &&
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

std::optional<Coord> AmbientMap::movement(const Coord &coord, int directionIndex) const{
    assert(directionIndex >= 0 && directionIndex < directionVector.size());
    auto neighbor = coord + directionVector[directionIndex];

    return isValid(neighbor) ? std::optional{neighbor} : std::nullopt;
}

AmbientMap::AmbientMap(const std::filesystem::path &gridPath, DistanceMatrix&& dm) :
    distanceMatrix{dm},
    grid{getGrid(openGridFile(gridPath))}
{
    if(dm.nRows != grid.size() || (grid.size() > 0 && dm.nCols != grid[0].size())){
        throw std::runtime_error("Grid file and distance matrix file do not refer to same ambient");
    }
}

int AmbientMap::getDistance(const Coord& a, const Coord& b) const{
    return distanceMatrix.getDistance(a,b);
}
