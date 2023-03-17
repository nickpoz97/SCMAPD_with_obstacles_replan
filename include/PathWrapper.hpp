#ifndef SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP

#include <unordered_set>
#include "Coord.hpp"
#include "Waypoint.hpp"
#include "NewTypes.hpp"

struct PathWrapper{
public:
    PathWrapper(Path path, WaypointsList  wpList, std::unordered_set<int> satisfiedTasksIds);
    PathWrapper() = default;
    PathWrapper(const PathWrapper&) = default;
    PathWrapper(PathWrapper&&) = default;

    bool removeTasksAndWaypoints(const std::unordered_set<int> &rmvTasksIndices);
    TimeStep getTTD() const;
    TimeStep getLastDeliveryTimeStep() const;

    const WaypointsList& getWaypoints() const;

    const Path& getPath() const;
    const std::unordered_set<int>& getSatisfiedTasksIds() const;

    CompressedCoord getInitialPos() const;
    void pathAndWaypointsUpdate(std::pair<Path, WaypointsList>&& updatedData);

    int randomTaskId(int magicNumber) const;

    PathWrapper& operator=(const PathWrapper& other) = default;
    PathWrapper& operator=(PathWrapper&& other) = default;
private:
    Path path;
    WaypointsList waypoints;

    std::pair<WaypointsList::iterator, WaypointsList::iterator> insertNewWaypoints(const Task &task, WaypointsList::iterator waypointStart,
                                                                                   WaypointsList::iterator waypointGoal);
    void restorePreviousWaypoints(WaypointsList::iterator waypointStart, WaypointsList::iterator waypointGoal);

    bool checkCapacityConstraint(int capacity) const;

    [[nodiscard]] TimeStep computeApproxTTD(const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                            WaypointsList::iterator newPickupWpIt) const ;
protected:
    std::unordered_set<int> satisfiedTasksIds;

    void
    insertTaskWaypoints(const Task &newTask, const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                        int agentCapacity);
};

class PWsVector : public std::vector<PathWrapper>{
public:
    [[nodiscard]] TimeStep getMaxSpanCost() const;
    [[nodiscard]] TimeStep getTTD() const;
    [[nodiscard]] TimeStep getTTT() const;
    [[nodiscard]] TimeStep getSpan(int agentId) const;
    [[nodiscard]] TimeStep getTasksDelay(int agentId) const;
    [[nodiscard]] const Path& getPath(int agentId) const;
};

struct ExtractedPath{
    int newTaskId;
    int agentId;
    PathWrapper wrapper;
};

inline std::size_t hash_value(const PathWrapper& p){
    return hash_value(p.getPath());
}

#endif //SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
