//
// Created by nicco on 29/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP
#define SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP

#include <filesystem>
#include <array>
#include "TypeDefs.hpp"
#include "Coord.hpp"

enum class CellType: char {
    ENDPOINT = 'G',
    OBSTACLE = '@',
    FLOOR = '.',
};

class AmbientMap {
public:
    static constexpr std::array<Movement,5> moves{{{0, -1}, {1, 0}, {0, 1}, {-1, 0}, {0, 0}}};

    explicit AmbientMap(const std::filesystem::path& gridPath);

    [[nodiscard]] bool isValid(Coord coord) const;
    CellType operator[](const Coord& coord) const;

    [[nodiscard]] int getNRows() const;

    [[nodiscard]] int getNCols() const;

    [[nodiscard]] std::vector<Coord> getNeighbors(const Coord& coord) const;

private:
    int nRows = 0;
    int nCols = 0;
    std::vector<std::vector<CellType>> grid;

    void fillMatrix(std::fstream& data);
};


#endif //SIMULTANEOUS_CMAPD_AMBIENTMAP_HPP
