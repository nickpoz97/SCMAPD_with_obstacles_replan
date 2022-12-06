#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <list>
#include <optional>

#include "Task.hpp"
#include "TypeDefs.hpp"
#include "Waypoint.hpp"
#include "Constraint.h"
#include "ambient/AmbientMapInstance.h"

class Assignment {

public:
    explicit Assignment(Coord startPosition, unsigned index, unsigned capacity);

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] TimeStep getMCA() const;

    [[nodiscard]] unsigned getIndex() const;

    [[nodiscard]] Coord getStartPosition() const;

    [[nodiscard]] const Path &getPath() const;

    [[nodiscard]] const WaypointsList &getWaypoints() const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] const std::vector<cmapd::Constraint> &getConstraints() const;

    void
    setTasks(WaypointsList &&newWaypoints, const std::vector<cmapd::Constraint> &outerConstraints,
             const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks);

    void
    insert(int taskId, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks,
           Heuristic heuristic);

    friend bool operator<(const Assignment &a, const Assignment &b);

    inline operator Coord() const { return getStartPosition(); }

    inline operator Path() const { return getPath(); }

    std::pair<Path, std::vector<cmapd::Constraint>> computePath(
            const cmapd::AmbientMapInstance &ambientMapInstance,
            const std::vector<cmapd::Constraint> &outerConstraints
    ) const;

    void recomputePath(const std::vector<cmapd::Constraint> &newConstraints, const Status &status);

    static bool hasConflicts(const Assignment& a, const Assignment& b);

    static bool checkConflict(const Path& a, const Path& b, int i);

    // this should be called when waypoints and/or constraints are changed
    void internalUpdate(const std::vector<cmapd::Constraint> &outerConstraints, const std::vector<Task> &tasks,
                        const cmapd::AmbientMapInstance &ambientMapInstance);
private:
    Coord startPosition;
    unsigned index;
    unsigned capacity;

    TimeStep oldTTD = 0;
    TimeStep newTTD = 0;

    std::vector<cmapd::Constraint> constraints{};
    WaypointsList waypoints{};
    Path path{};

    void insertTaskWaypoints(const Task &task, WaypointsList::iterator &waypointStart,
                             WaypointsList::iterator &waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(WaypointsList::iterator &waypointStart,
                                  WaypointsList::iterator &waypointGoal);

    [[nodiscard]] TimeStep computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix,
                                          WaypointsList::const_iterator lastWaypoint, int firstIndexPath = 0) const;

    [[nodiscard]] TimeStep computeRealTTD(
            const std::vector<Task> &tasks,
            const DistanceMatrix &distanceMatrix
    ) const;

    [[nodiscard]] TimeStep computeApproxTTD(
        const std::vector<Task> &tasks,
        const DistanceMatrix &distanceMatrix,
        WaypointsList::const_iterator startWaypoint,
        WaypointsList::const_iterator goalWaypoint
    ) const;

    // first index has been added to reduce search time
    static std::optional<TimeStep> findWaypointTimestep(const Path &path, const Waypoint &waypoint, int firstIndex = 0);

    std::pair<WaypointsList::iterator, WaypointsList::iterator>
    findBestPositions(int taskId, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks);
};
#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
