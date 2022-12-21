/// @file Assignment.hpp

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <list>
#include <optional>

#include "Task.hpp"
#include "TypeDefs.hpp"
#include "Waypoint.hpp"
#include "Constraint.h"
#include "ambient/AmbientMapInstance.h"

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
    explicit Assignment(Coord startPosition, int index, int capacity);

    /// @return agent capacity
    [[nodiscard]] int getCapacity() const;

    /// @return difference between actual TTD and TTD before last task addition to path
    [[nodiscard]] TimeStep getMCA() const;

    /// @return agent numerical id
    [[nodiscard]] int getIndex() const;

    /// @return agent initial position
    [[nodiscard]] Coord getStartPosition() const;

    /// @return reference to agent path
    [[nodiscard]] const Path &getPath() const;

    /// @return true if agent contains no waypoints
    [[nodiscard]] bool empty() const;

    /**
     * @param instance ambient used to compute constraints together with path
     * @return constraints which other agents must avoid
     * @warning linear complexity (wrt path length)
     */
    [[nodiscard]] std::vector<cmapd::Constraint>
    getConstraints(const cmapd::AmbientMapInstance &instance) const;

    /**
     * @brief add a new task in the best position and recompute path
     * @param taskId
     * @param ambientMapInstance
     * @param tasks
     * @param constraints
     */
    void
    addTask(const cmapd::AmbientMapInstance &ambientMapInstance,
            const std::vector<std::vector<cmapd::Constraint>> &constraints, int taskId,
            const std::vector<Task> &tasks, const std::vector<Assignment> &actualAssignments);

    friend bool operator<(const Assignment &a, const Assignment &b);

    inline explicit operator Coord() const { return getStartPosition(); }

    inline explicit operator Path() const { return getPath(); }

    // this should be called when waypoints and/or constraints are changed
    void internalUpdate(const std::vector<std::vector<cmapd::Constraint>> &constraints,
                        const std::vector<Task> &tasks, const cmapd::AmbientMapInstance &ambientMapInstance,
                        bool newTasks, const std::vector<Assignment> &actualAssignments);

    explicit operator std::string() const;

    static bool conflictsWithPath(const Path &a, const Path &b);
    static bool checkConflictAtTime(const Path &a, const Path &b, TimeStep i);
    [[nodiscard]] bool conflictsWithOthers(const std::vector<Assignment>& actualAssignments) const;

    static inline const cmapd::moves_t moves{{0, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, 0}};
private:
    Coord startPosition;
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
};
std::vector<Assignment> loadAssignments(const std::filesystem::path &agentsFilePath, char horizontalSep=',', int capacity=3);

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
