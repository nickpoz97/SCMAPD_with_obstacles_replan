/// @file Assignment.hpp

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <list>
#include <optional>

#include "Task.hpp"
#include "TypeDefs.hpp"
#include "Waypoint.hpp"
#include "Status.hpp"

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
    explicit Assignment(CompressedCoord startPosition, int index, int capacity);

    /// @return agent capacity
    [[nodiscard]] int getCapacity() const;

    /// @return difference between actual TTD and TTD before last task addition to path
    [[nodiscard]] TimeStep getMCA() const;

    /// @return agent numerical id
    [[nodiscard]] int getIndex() const;

    /// @return agent initial position
    [[nodiscard]] CompressedCoord getStartPosition() const;

    /// @return actual path
    /// @warning actual path is cleared
    [[nodiscard]] Path && extractPath();

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

    friend bool operator<(const Assignment &a, const Assignment &b);

    inline explicit operator Coord() const { return getStartPosition(); }

    // this should be called when waypoints and/or constraints are changed
    void internalUpdate(const std::vector<Task> &tasks, const Status &status);

    static bool conflictsWithPath(const Path &a, const Path &b);
    static bool checkConflictAtTime(const Path &a, const Path &b, TimeStep i);
    [[nodiscard]] bool conflictsWithOthers(const std::vector<Assignment>& actualAssignments) const;

    [[nodiscard]] const WaypointsList &getWaypoints() const;
private:
    CompressedCoord startPosition;
    int index;
    int capacity;

    TimeStep oldTTD = 0;
    TimeStep newTTD = 0;

    WaypointsList waypoints{};
    Path path{};

    std::pair<WaypointsList::iterator, WaypointsList::iterator> insertNewWaypoints(const Task &task, std::_List_iterator<Waypoint> waypointStart,
                                                                                   std::_List_iterator<Waypoint> waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(std::_List_iterator<Waypoint> waypointStart,
                                  std::_List_iterator<Waypoint> waypointGoal);

    [[nodiscard]] TimeStep computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix,
                                          const std::_List_iterator<Waypoint> &firstWaypoint,
                                          const std::_List_iterator<Waypoint> &lastWaypoint);

    [[nodiscard]] TimeStep computeRealTTD(
            const std::vector<Task> &tasks,
            const DistanceMatrix &distanceMatrix
    );

    [[nodiscard]] TimeStep computeApproxTTD(
            const Task &task,
            const DistanceMatrix &distanceMatrix,
            std::_List_iterator<Waypoint> startWaypoint,
            std::_List_iterator<Waypoint> goalWaypoint
    )const ;

    std::pair<WaypointsList::iterator, WaypointsList::iterator>
    findBestPositions(const Task &task, const DistanceMatrix &distanceMatrix);

    static bool checkConflictAtTime(const Path &a, const std::pair<Coord, Coord> &b, TimeStep i);
};
std::vector<Assignment> loadAssignments(const std::filesystem::path &agentsFilePath, char horizontalSep=',', int capacity=3);

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
