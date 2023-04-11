/// @file Assignment.hpp

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <list>
#include <optional>

#include "Task.hpp"
#include "NewTypes.hpp"
#include "Waypoint.hpp"
#include "Status.hpp"
#include "AgentInfo.hpp"
#include "PathWrapper.hpp"

/**
 * @class Assignment
 * @brief class that abstracts an agent and its path with other infos
 */
class Assignment : public PathWrapper{

public:
    explicit Assignment(const AgentInfo &agentInfo);
    explicit Assignment(const PathWrapper& pW);

    [[nodiscard]] TimeStep getMCA() const;

    [[nodiscard]] bool addTask(int taskId, const Status &status);

//    friend bool operator>(const Assignment &a, const Assignment &b);
//    friend bool operator<(const Assignment &a, const Assignment &b);
    friend int operator<=>(const Assignment &a, const Assignment &b);

    // this should be called when waypoints and/or constraints are changed
    [[nodiscard]] bool internalUpdate(const Status &status);

    bool removeTasksAndWaypoints(const std::unordered_set<int> &rmvTasksIndices, const Status& status);
private:
    TimeStep oldTTD = 0;

    std::pair<WaypointsList::iterator, WaypointsList::iterator> insertNewWaypoints(const Task &task, WaypointsList::iterator waypointStart,
                                                                                   WaypointsList::iterator waypointGoal);
    void restorePreviousWaypoints(WaypointsList::iterator waypointStart, WaypointsList::iterator waypointGoal);

    bool checkCapacityConstraint(int capacity) const;

    [[nodiscard]] TimeStep computeApproxTTD(const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                            WaypointsList::const_iterator newPickupWpIt) const ;

    [[nodiscard]] TimeStep computeApproxSpan(const DistanceMatrix &dm, WaypointsList::const_iterator startIt) const;
    [[nodiscard]] TimeStep computeIdealTTD(const DistanceMatrix &dm, const std::vector<Task> &tasks) const;

    TimeStep computeIdealCost(const Status &status) const;

    TimeStep
    insertTaskWaypoints(const Task &newTask, const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
        int agentCapacity);

    void updateWaypointsStats(const std::vector<Task>& tasksVector);
};

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
