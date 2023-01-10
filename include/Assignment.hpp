/// @file Assignment.hpp

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <list>
#include <optional>

#include "Task.hpp"
#include "TypeDefs.hpp"
#include "Waypoint.hpp"
#include "Status.hpp"
#include "AgentInfo.hpp"

struct PathWrapper{
    int agentId;
    Path path;
};

/**
 * @class Assignment
 * @brief class that abstracts an agent and its path
 */
class Assignment {

public:
    /**
     *
     * @param startPosition initial position of the agent
     * @param index numerical id for the agent
     * @param capacity max number of tasks the agent can keep
     */
    explicit Assignment(const AgentInfo &agentInfo, int firstTaskId, const Status &status);

    /// @return agent capacity
    [[nodiscard]] int getCapacity() const;

    /// @return difference between actual TTD and TTD before last task addition to path
    [[nodiscard]] TimeStep getMCA() const;

    /// @return agent numerical id
    [[nodiscard]] int getAgentId() const;

    /// @return agent initial position
    [[nodiscard]] CompressedCoord getStartPosition() const;

    /// @return actual path
    /// @warning actual path is cleared
    [[nodiscard]] PathWrapper extractAndReset();

    /// @return true if agent contains no waypoints
    [[nodiscard]] bool empty() const;

    /**
     * @brief add a new task in the best position and recompute path
     * @param taskId
     * @param ambientMapInstance
     * @param tasks
     * @param constraints
     */
    void
    addTask(int taskId, const Status &status);

    [[nodiscard]] const Path& getPath() const;

    friend bool operator<(const Assignment &a, const Assignment &b);
    friend bool operator>(const Assignment &a, const Assignment &b);

    inline explicit operator CompressedCoord() const { return getStartPosition(); }

    // this should be called when waypoints and/or constraints are changed
    void internalUpdate(const Status &status);

    [[nodiscard]] const WaypointsList &getWaypoints() const;
private:
    CompressedCoord startPos;
    int index;
    int capacity;

    TimeStep oldTTD = 0;

    WaypointsList waypoints{};
    Path path{};

    std::pair<WaypointsList::iterator, WaypointsList::iterator> insertNewWaypoints(const Task &task, std::_List_iterator<Waypoint> waypointStart,
                                                                                   std::_List_iterator<Waypoint> waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(std::_List_iterator<Waypoint> waypointStart,
                                  std::_List_iterator<Waypoint> waypointGoal);

    [[nodiscard]] TimeStep getActualTTD() const;

    [[nodiscard]] TimeStep computeApproxTTD(const Status &status, WaypointsList::iterator newPickupWpIt,
                                            WaypointsList::iterator newDeliveryWpIt) const ;

    void
    insertTaskWaypoints(int taskId, const Status &status);

};

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
