#ifndef SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP

#include <unordered_set>
#include "Coord.hpp"
#include "Waypoint.hpp"
#include "NewTypes.hpp"
#include "AgentInfo.hpp"

struct PathWrapper{
public:
    PathWrapper(Path path, WaypointsList  wpList, std::unordered_set<int> satisfiedTasksIds);
    explicit PathWrapper(const AgentInfo& agentInfo);
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
    [[nodiscard]] TimeStep getIdealCost() const;
    [[nodiscard]] TimeStep getActualTTD() const;
private:
    Path path;
    WaypointsList waypoints;

    bool checkCapacityConstraint() const;

    [[nodiscard]] TimeStep computeApproxTTD(const DistanceMatrix &dm, const std::vector<Task> &tasksVector,
                                            std::_List_const_iterator<Waypoint> newPickupWpIt) const ;
protected:
    int capacity{};
    std::unordered_set<int> satisfiedTasksIds;
    TimeStep idealCost{};
    TimeStep ttd{};
    TimeStep realCost{};

    void
    insertTaskWaypoints(const Task &newTask, const DistanceMatrix &dm, const std::vector<Task> &tasksVector);

    TimeStep computeIdealCost(const DistanceMatrix &dm) const;
    TimeStep computeTTD(const std::vector<Task> &tasks) const;
};

class PWsVector : public std::vector<PathWrapper>{
public:
    [[nodiscard]] TimeStep getMaxSpanCost() const;
    [[nodiscard]] TimeStep getTTD() const;
    [[nodiscard]] TimeStep getTTT() const;
    [[nodiscard]] TimeStep getSpan(int agentId) const;
    [[nodiscard]] TimeStep getTasksDelay(int agentId) const;
    [[nodiscard]] const Path& getPath(int agentId) const;
    [[nodiscard]] TimeStep getIdealCost() const;
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
