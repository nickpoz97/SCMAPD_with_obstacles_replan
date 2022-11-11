//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_TARGET_HPP
#define SIMULTANEOUS_CMAPD_TARGET_HPP

#include <TypeDefs.hpp>

class Target {
public:
    Target(LocationIndex startLoc, LocationIndex goalLoc, TimeStamp releaseTime);

    LocationIndex getStartLoc() const;

    LocationIndex getGoalLoc() const;

    TimeStamp getReleaseTime() const;

private:
    const LocationIndex startLoc;
    const LocationIndex  goalLoc;
    const TimeStamp releaseTime;
};


#endif //SIMULTANEOUS_CMAPD_TARGET_HPP
