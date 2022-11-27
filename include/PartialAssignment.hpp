//
// Created by nicco on 11/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP

#include <TypeDefs.hpp>
#include <list>
#include "Task.hpp"
#include "Assignment.hpp"

class PartialAssignment : public Assignment {
public:
    PartialAssignment(const Assignment& assignment) : Assignment{assignment} {};
    [[nodiscard]] const WaypointsList &getWaypoints() const;

    [[nodiscard]] bool empty() const;

    // todo setTasks must not need newTtd
    void setTasks(WaypointsList &&newWaypoints, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);
    void setTasks(const WaypointsList &newWaypoints, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);

    void setTasks(PartialAssignment &&pa, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);
    void setTasks(const PartialAssignment &pa, const TasksVector &tasks, const DistanceMatrix &distanceMatrix);

    void insert(const Task &task, Heuristic heuristic, const DistanceMatrix &distanceMatrix, const TasksVector &tasks);
private:
    WaypointsList waypoints{};

    void insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                             WaypointsList::iterator &waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                  WaypointsList::iterator &waypointGoal);

    [[nodiscard]] TimeStep computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const;

    [[nodiscard]] TimeStep computeApproxTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix) const;

    void updatePath();
};

#endif //SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
