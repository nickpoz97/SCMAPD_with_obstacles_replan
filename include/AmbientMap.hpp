//
// Created by nicco on 29/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP
#define SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP

#include <filesystem>
#include <array>
#include <optional>
#include "TypeDefs.hpp"
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

    AmbientMap(const std::filesystem::path &gridPath, DistanceMatrix&& dm);

    [[nodiscard]] bool isValid(const Coord &coord) const;
    CellType operator[](const Coord& coord) const;

    [[nodiscard]] int getNRows() const;

    [[nodiscard]] int getNCols() const;

    [[nodiscard]] std::optional<CompressedCoord> movement(CompressedCoord coord, int directionIndex) const;

    [[nodiscard]] const DistanceMatrix &getDistanceMatrix() const;

private:
    const DistanceMatrix& distanceMatrix;
    std::vector<std::vector<CellType>> grid;

    static std::vector<std::vector<CellType>> getGrid(std::fstream &&data);
    static std::fstream openGridFile(const std::filesystem::path &gridPath);

};

#endif //SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP
