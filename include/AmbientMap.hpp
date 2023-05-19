//
// Created by nicco on 29/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP
#define SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP

#include <filesystem>
#include <array>
#include <optional>
#include "NewTypes.hpp"
#include "Coord.hpp"
#include "DistanceMatrix.hpp"
#include <boost/iterator/counting_iterator.hpp>

enum class CellType: char {
    ENDPOINT = 'G',
    OBSTACLE = '@',
    FLOOR = '.',
};


class AmbientMap {
public:
    static constexpr std::array<Direction,5> directionVector{{{-1, 0}, {0, 1}, {1, 0}, {0, -1}, {0, 0}}};
    static constexpr int nDirections = directionVector.size();
    static constexpr int getHoldDirectionIndex();

    AmbientMap(const std::filesystem::path &gridPath, const std::filesystem::path &distanceMatrixPath);

    [[nodiscard]] bool isValid(const Coord &coord) const;
    CellType operator[](CompressedCoord cc) const;

    [[nodiscard]] int getNRows() const;
    [[nodiscard]] int getNCols() const;
    [[nodiscard]] const DistanceMatrix &getDistanceMatrix() const;

    [[nodiscard]] std::optional<CompressedCoord> movement(CompressedCoord coord, int directionIndex) const;
    [[nodiscard]] const std::vector<CellType>& getGrid() const;
    [[nodiscard]] std::vector<std::string> getRowsStrings() const;
private:
    const DistanceMatrix distanceMatrix;
    std::vector<CellType> grid;
    static std::vector<CellType> loadGrid(const std::filesystem::path &gridPath);
};

constexpr int AmbientMap::getHoldDirectionIndex() {
    for(int i = 0 ; i < nDirections ; ++i){
        if(directionVector[i] == Direction{0,0}){
            return i;
        }
    }
    return -1;
};

#endif //SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP
