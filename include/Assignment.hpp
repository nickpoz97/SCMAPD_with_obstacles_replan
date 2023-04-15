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
    Assignment(const PathWrapper &pW, Status &status);

    [[nodiscard]] TimeStep getMCA() const;

    [[nodiscard]] bool addTask(int taskId);

    friend int operator<=>(const Assignment &a, const Assignment &b);

    // this should be called when waypoints and/or constraints are changed
    [[nodiscard]] bool internalUpdate();

    bool removeTasksAndWaypoints(const std::unordered_set<int> &rmvTasksIndices);
private:
    TimeStep oldTTD = 0;
    Status& status;

    bool checkCapacityConstraint() const;

    [[nodiscard]] TimeStep computeApproxTTD(WaypointsList::const_iterator newPickupWpIt) const ;

    [[nodiscard]] TimeStep computeApproxSpan(WaypointsList::const_iterator startIt) const;
    [[nodiscard]] TimeStep computeIdealTTD() const;

    TimeStep computeIdealCost() const;

    TimeStep
    insertTaskWaypoints(int newTaskId);

    void updateWaypointsStats();
};

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
