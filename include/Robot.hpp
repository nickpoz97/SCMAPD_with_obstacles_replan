//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ROBOT_HPP
#define SIMULTANEOUS_CMAPD_ROBOT_HPP

#include <TypeDefs.hpp>
#include <list>
#include "Task.hpp"

class Robot {
public:
    explicit Robot(CompressedCoord position, unsigned index, unsigned capacity);
    Robot(const Robot& robot) = default;

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] const WaypointsList &getWaypoints() const;

    [[nodiscard]] TimeStep getTtd() const;

    [[nodiscard]] unsigned getIndex() const;

    [[nodiscard]] CompressedCoord getStartPosition() const;

    [[nodiscard]] bool empty() const;

    void setTasksAndTTD(WaypointsList &&newWaypoints, TimeStep newTtd);
    void setTasksAndTTD(const WaypointsList &newWaypoints, TimeStep newTtd);

    void setTasksAndTTD(Robot &&robot);
    void setTasksAndTTD(const Robot &robot);

    void insert(const Task& task, Heuristic heuristic);
private:
    const CompressedCoord startPosition;
    const unsigned capacity;
    const unsigned index;

    WaypointsList waypoints{};
    TimeStep ttd = 0;

    void insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                             WaypointsList::iterator &waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                  WaypointsList::iterator &waypointGoal);

    TimeStep updateBestWaypoints(TimeStep bestTTD, WaypointsList::iterator &bestStart, WaypointsList::iterator &bestEnd);
};

#endif //SIMULTANEOUS_CMAPD_ROBOT_HPP
