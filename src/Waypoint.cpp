#include "Waypoint.hpp"

cmapd::Point Waypoint::toPoint(int nCols) const{
    // row, col
    return {
        static_cast<int>(position) / nCols,
        static_cast<int>(position) % nCols
    };
}