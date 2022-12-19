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

    [[nodiscard]] std::vector<cmapd::Constraint>
    getConstraints(const cmapd::AmbientMapInstance &instance) const;

    void
    insert(int taskId, const cmapd::AmbientMapInstance &ambientMapInstance, const std::vector<Task> &tasks,
           const std::vector<std::vector<cmapd::Constraint>> &constraints);

    friend bool operator<(const Assignment &a, const Assignment &b);

    inline explicit operator Coord() const { return getStartPosition(); }

    inline explicit operator Path() const { return getPath(); }

    // this should be called when waypoints and/or constraints are changed
    void internalUpdate(const std::vector<std::vector<cmapd::Constraint>> &constraints, const std::vector<Task> &tasks,
                        const cmapd::AmbientMapInstance &ambientMapInstance, bool newTasks);

    explicit operator std::string() const;

    [[nodiscard]] bool hasConflicts(const std::vector<cmapd::Constraint> &constraints) const;

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
            const std::vector<Task> &tasks,
            const DistanceMatrix &distanceMatrix,
            std::_List_iterator<Waypoint> startWaypoint,
            std::_List_iterator<Waypoint> goalWaypoint
    )const ;

    // first index has been added to reduce search time
    static std::optional<TimeStep> findWaypointTimestep(const Path &path, const Waypoint &waypoint);

    std::pair<WaypointsList::iterator, WaypointsList::iterator>
    findBestPositions(int taskId, const DistanceMatrix &distanceMatrix, const std::vector<Task> &tasks);

    static bool conflictsWith(const Path & path, TimeStep i, const cmapd::Constraint& c);
};
std::vector<Assignment> loadAssignments(const std::filesystem::path &agentsFilePath, int nCols, char horizontalSep= ',', int capacity= 3);

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
