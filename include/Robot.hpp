//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ROBOT_HPP
#define SIMULTANEOUS_CMAPD_ROBOT_HPP

#include <TypeDefs.hpp>
#include "Task.hpp"

using Waypoints = std::list<CompressedCoord>;

class Robot {
public:
    explicit Robot(CompressedCoord position, unsigned index, unsigned capacity);
    Robot(const Robot& robot) = default;

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] const Waypoints &getWaypoints() const;

    [[nodiscard]] TimeStep getTtd() const;

    [[nodiscard]] unsigned getIndex() const;

    [[nodiscard]] CompressedCoord getPosition() const;

    bool empty() const;

    void setTasksAndTTD(Waypoints &&newActions, TimeStep newTtd);

    Waypoints releaseWaypoints();
private:
    const CompressedCoord position;
    const unsigned capacity;
    const unsigned index;

    Waypoints waypoints{};
    TimeStep ttd = 0;
};

using RobotVector = std::vector<Robot>;

#endif //SIMULTANEOUS_CMAPD_ROBOT_HPP
