//
// Created by nicco on 26/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP


#include "Task.hpp"
#include <list>
#include "TypeDefs.hpp"
#include "Waypoint.hpp"

class Assignment {

public:
    explicit Assignment(CompressedCoord startPosition, unsigned index, unsigned capacity);

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] TimeStep getTtd() const;

    [[nodiscard]] unsigned getIndex() const;

    [[nodiscard]] CompressedCoord getStartPosition() const;

    [[nodiscard]] const Path &getPath() const;

    void update(Assignment&& assignment);

    [[nodiscard]] const WaypointsList &getWaypoints() const;

    [[nodiscard]] bool empty() const;

    void setTasks(WaypointsList &&newWaypoints, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);
    void setTasks(const WaypointsList &newWaypoints, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);

    void setTasks(Assignment &&pa, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);
    void setTasks(const Assignment &pa, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);

    void insert(const Task &task, Heuristic heuristic, const DistanceMatrix &distanceMatrix, const TasksVector &tasks);

    friend bool operator<(const Assignment& a, const Assignment& b);

private:
    CompressedCoord startPosition;
    unsigned capacity;
    unsigned index;

    TimeStep ttd = 0;
    WaypointsList waypoints{};
    unsigned sortKey = 0;

    void insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                             WaypointsList::iterator &waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                  WaypointsList::iterator &waypointGoal);

    [[nodiscard]] TimeStep computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const;

    [[nodiscard]] TimeStep computeApproxTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const;

    void updatePath();

    static constexpr unsigned computeHeuristic(Heuristic heur, TimeStep newtOk, TimeStep oldtOk);
};


#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
