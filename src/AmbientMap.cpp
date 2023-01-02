
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "AmbientMap.hpp"

AmbientMap::AmbientMap(const std::filesystem::path &gridPath) {
    std::fstream fs(gridPath.c_str(), std::ios::in);

    if(!fs.is_open()){
        throw std::runtime_error("Grid file doesn' t exist");
    }
    fillMatrix(fs);
}

void AmbientMap::fillMatrix(std::fstream &data){
    std::string line;
    std::getline(data, line);

    if(line.empty()){
        throw std::runtime_error("Empty grid file");
    }

    char horizontalSep = ',';

    std::stringstream lineStream{line};
    std::string token;

    std::getline(lineStream, token, horizontalSep);
    nRows = std::stoi(token);

    std::getline(lineStream, token, horizontalSep);
    nCols = std::stoi(token);

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
    bool isInsideGrid = coord.row <= nRows && coord.col <= nCols &&
        coord.row >= 0 && coord.col >= 0;

    return isInsideGrid && operator[](coord) != CellType::OBSTACLE;
}

CellType AmbientMap::operator[](const Coord &coord) const {
    return grid[coord.row][coord.col];
}

int AmbientMap::getNRows() const {
    return nRows;
}

int AmbientMap::getNCols() const {
    return nCols;
}

std::vector<Coord> AmbientMap::getNeighbors(const Coord& coord) const{
    std::vector<Coord> neighbors;
    for(const auto& mv : moves){
        auto neighbor = coord + mv;
        if(isValid(neighbor)){
            neighbors.push_back(neighbor);
        }
    }
    return neighbors;
}
