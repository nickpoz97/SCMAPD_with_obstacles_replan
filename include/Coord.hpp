//
// Created by nicco on 02/01/2023.
//

#ifndef SIMULTANEOUS_CMAPD_COORD_HPP
#define SIMULTANEOUS_CMAPD_COORD_HPP


#include <vector>
#include <string>
#include <boost/container_hash/hash.hpp>
#include "NewTypes.hpp"
#include <nlohmann/json.hpp>

struct Coord {
    int row;
    int col;

    using Direction = Coord;
    friend Coord operator+(const Coord& coord, const Direction& movement);
    friend bool operator==(const Coord& a, const Coord& b) = default;

    explicit operator nlohmann::json() const;
};
using Direction = Coord::Direction;

using CompressedCoord = int;

using Path = std::vector<CompressedCoord>;
inline std::size_t hash_value(const Path& p){ return boost::hash_range(p.cbegin(), p.cend()); }

class VerbosePath : public std::vector<Coord>{
public:
    explicit operator nlohmann::json() const;
};

VerbosePath getVerbosePath(Path path, int nCols);

#endif //SIMULTANEOUS_CMAPD_COORD_HPP
