//
// Created by nicco on 11/11/2022.
//

#include "Robot.hpp"

Robot::Robot(LocationIndex start) :
    start{start}
    {}

LocationIndex Robot::getStart() const {
    return start;
}
