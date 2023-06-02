//
// Created by nicco on 02/01/2023.
//

#include "Coord.hpp"
#include "fmt/ranges.h"
#include "fmt/core.h"

Coord operator+(const Coord& coord, const Direction& movement){
    return {coord.row + movement.row, coord.col + movement.col};
}

Coord::operator nlohmann::json() const {
    return nlohmann::json::array({row, col});
}

template <> struct fmt::formatter<Coord>{
    template <typename ParseContext>
    constexpr auto parse(ParseContext& ctx) {
        return ctx.begin();
    }

    template <typename FormatContext>
    auto format(const Coord& c, FormatContext& ctx) const{
        return format_to(ctx.out(), "({},{})", c.row, c.col);
    }
};

VerbosePath::operator nlohmann::json() const {
    nlohmann::json j;
    for (const auto& coord : *this){
        j.push_back(nlohmann::json(coord));
    }
    return j;
}

VerbosePath getVerbosePath(Path path, int nCols) {
    VerbosePath vp{};
    vp.reserve(path.size());

    std::ranges::transform(
            path,
            std::back_inserter(vp),
            [nCols](CompressedCoord cc) -> Coord {return {cc / nCols, cc % nCols};}
    );

    return vp;
}
