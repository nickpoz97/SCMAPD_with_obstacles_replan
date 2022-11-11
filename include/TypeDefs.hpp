//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
#define SIMULTANEOUS_CMAPD_TYPEDEFS_HPP

#include <vector>
#include <list>

using Coord = unsigned int;
using LocationIndex = unsigned int;
using TimeStamp = unsigned int;

using SequenceOfActions = std::list<LocationIndex>;
using AssignmentsPerRobot = std::vector<SequenceOfActions>;

#endif //SIMULTANEOUS_CMAPD_TYPEDEFS_HPP
