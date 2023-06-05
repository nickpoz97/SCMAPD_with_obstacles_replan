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

bool Path::hasConflict(CompressedCoord coord1, CompressedCoord coord2, TimeStep t1, bool isFinal) const {
    // if path is empty there are no conflicts
    assert(!this->empty());
    TimeStep lastTimeStep = static_cast<TimeStep>(size()) - 1;

    auto t2 = std::min(t1 + 1, lastTimeStep);
    assert(t2 >= 0);

    bool nodeConflict = coord2 == operator[](t2);
    bool edgeConflict = t1 < lastTimeStep && coord1 == operator[](t2) && coord2 == operator[](t1);
    bool dockConflict = isFinal && t2 <= lastTimeStep &&
        std::ranges::any_of(cbegin() + t2, cend(), [coord2](CompressedCoord cc){return cc == coord2;});

    return nodeConflict || edgeConflict || dockConflict;
}
