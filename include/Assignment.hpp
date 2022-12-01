//
// Created by nicco on 26/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP


#include "Task.hpp"
#include <list>
#include "TypeDefs.hpp"
#include "Waypoint.hpp"
#include "ambient/AmbientMapInstance.h"

class Assignment {

public:
    explicit Assignment(Coord startPosition, unsigned index, unsigned capacity);

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] TimeStep getTtd() const;

    [[nodiscard]] unsigned getIndex() const;

    [[nodiscard]] Coord getStartPosition() const;

    [[nodiscard]] const Path &getPath() const;

    void update(Assignment&& assignment);

    [[nodiscard]] const WaypointsList &getWaypoints() const;

    [[nodiscard]] bool empty() const;

    void setTasks(WaypointsList &&newWaypoints, const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix);
    void setTasks(const WaypointsList &newWaypoints, const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix);

    void setTasks(Assignment &&pa, const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix);
    void setTasks(const Assignment &pa, const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix);

    void insert(const Task &task, Heuristic heuristic, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks);

    friend bool operator<(const Assignment& a, const Assignment& b);

    inline operator Coord() const {return getStartPosition();}
    inline operator Path() const {return getPath();}

    Path computePathAndTTD(
        const std::vector<Assignment> &assignments,
        const cmapd::AmbientMapInstance &ambientMapInstance,
        bool usePbs
    ) const;
private:
    Coord startPosition;
    unsigned capacity;
    unsigned index;

    TimeStep ttd = 0;
    WaypointsList waypoints{};
    Path path{};
    unsigned sortKey = 0;

    void insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                             WaypointsList::iterator &waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                  WaypointsList::iterator &waypointGoal);

    [[nodiscard]] TimeStep computeRealTTD(
        const std::vector<Task> &tasks,
        const DistanceMatrix &distanceMatrix,
        WaypointsList::const_iterator lastWaypoint
    ) const;

    [[nodiscard]] TimeStep computeRealTTD(
            const std::vector<Task> &tasks,
            const DistanceMatrix &distanceMatrix
    ) const;

    [[nodiscard]] TimeStep computeApproxTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const;

    static std::pair<Path, TimeStep>
    computePathAndTTD(Coord initialPos, const WaypointsList &waypointsList, bool usePbs);
};


#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
