#ifndef SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
#define SIMULTANEOUS_CMAPD_TYPEDEFS_HPP

#include <vector>
#include <list>
#include <queue>

using Coord = std::pair<unsigned int, unsigned int>;
using CompressedCoord = unsigned int;
using TimeStamp = unsigned int;

using PickupDropoffTime = std::pair<TimeStamp, TimeStamp>;

using DistanceMatrix = std::vector<std::vector<TimeStamp>>;

#endif //SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
