//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ROBOT_HPP
#define SIMULTANEOUS_CMAPD_ROBOT_HPP

#include <TypeDefs.hpp>

class Robot {
public:
    explicit Robot(LocationIndex start);

    LocationIndex getStart() const;

private:
    const LocationIndex start;
};


#endif //SIMULTANEOUS_CMAPD_ROBOT_HPP
