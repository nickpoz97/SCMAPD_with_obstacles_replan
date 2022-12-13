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
    explicit Assignment(Coord startPosition, int index, int capacity);

    [[nodiscard]] int getCapacity() const;

    [[nodiscard]] TimeStep getMCA() const;

    [[nodiscard]] int getIndex() const;

    [[nodiscard]] Coord getStartPosition() const;

    [[nodiscard]] const Path &getPath() const;

    [[nodiscard]] const WaypointsList &getWaypoints() const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] const std::vector<cmapd::Constraint> &getConstraints() const;

    void
    insert(int taskId, const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks,
           const std::vector<cmapd::Constraint> &outerConstraints);

    friend bool operator<(const Assignment &a, const Assignment &b);

    inline explicit operator Coord() const { return getStartPosition(); }

    inline explicit operator Path() const { return getPath(); }

    static bool hasConflicts(const Assignment& a, const Assignment& b);

    static bool checkConflict(const Path& a, const Path& b, int i);

    // this should be called when waypoints and/or constraints are changed
    void internalUpdate(const std::vector<cmapd::Constraint> &outerConstraints, const std::vector<Task> &tasks,
                        const cmapd::AmbientMapInstance &ambientMapInstance);

    explicit operator std::string() const;
private:
    Coord startPosition;
    int index;
    int capacity;

    TimeStep oldTTD = 0;
    TimeStep newTTD = 0;

    std::vector<cmapd::Constraint> constraints{};
    WaypointsList waypoints{};
    Path path{};

    std::pair<WaypointsList::iterator, WaypointsList::iterator> insertNewWaypoints(const Task &task, std::_List_iterator<Waypoint> waypointStart,
                                                                                   std::_List_iterator<Waypoint> waypointGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(std::_List_iterator<Waypoint> waypointStart,
                                  std::_List_iterator<Waypoint> waypointGoal);

    [[nodiscard]] TimeStep computeRealTTD(const std::vector<Task> &tasks, const DistanceMatrix &distanceMatrix,
                                          WaypointsList::const_iterator firstWaypoint,
                                          WaypointsList::const_iterator lastWaypoint) const;

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
    static std::optional<TimeStep> findWaypointTimestep(const Path &path, const Waypoint &waypoint);

    std::pair<WaypointsList::iterator, WaypointsList::iterator>
    findBestPositions(int taskId, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks);
};

std::vector<Assignment> loadAssignments(const std::filesystem::path &agentsFilePath, int nCols, char horizontalSep= ',', int capacity= 3);

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
