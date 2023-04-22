#ifndef SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP

#include <unordered_set>
#include "Coord.hpp"
#include "Waypoint.hpp"
#include "NewTypes.hpp"
#include "AgentInfo.hpp"

struct PathWrapper{
public:
    explicit PathWrapper(const AgentInfo &agentInfo);
    PathWrapper(const PathWrapper&) = default;
    PathWrapper(PathWrapper&&) = default;

    TimeStep getTTD() const;
    TimeStep getLastDeliveryTimeStep() const;

    const WaypointsList& getWaypoints() const;

    const Path& getPath() const;
    const std::unordered_set<int>& getSatisfiedTasksIds() const;

    void extendAndReset(TimeStep actualTimeStep);

    int randomTaskId(int magicNumber) const;

    PathWrapper& operator=(const PathWrapper&) = default;
    PathWrapper& operator=(PathWrapper&&) = default;

    [[nodiscard]] TimeStep getIdealTTD() const;

    [[nodiscard]] int getAgentId() const;
    [[maybe_unused]] [[nodiscard]] int getCapacity() const;

    [[nodiscard]] bool empty() const;

    [[nodiscard]] bool isAvailable(TimeStep t) const;
private:
    int index;
    int capacity;
    Path path;

    TimeStep idealTTD;
protected:
    WaypointsList waypoints;
    std::unordered_set<int> satisfiedTasksIds;

    void updatePath(const Path &newPath, TimeStep connectionTimeStep);

    void setIdealTtd(TimeStep idealTtd);
};

class PWsVector : public std::vector<PathWrapper>{
public:
    [[nodiscard]] TimeStep getMaxSpanCost() const;
    [[nodiscard]] TimeStep getTTD() const;
    [[nodiscard]] TimeStep getTTT() const;
    [[nodiscard]] TimeStep getSpan(int agentId) const;
    [[nodiscard]] TimeStep getTasksDelay(int agentId) const;
    [[nodiscard]] const Path& getPath(int agentId) const;
    [[nodiscard]] bool taskIsSatisfied(int taskId) const;
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
