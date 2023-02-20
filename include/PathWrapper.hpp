#ifndef SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
#define SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP

#include <unordered_set>
#include "Coord.hpp"
#include "Waypoint.hpp"
#include "NewTypes.hpp"

struct PathWrapper{
public:
    PathWrapper(Path path, WaypointsList  wpList, std::unordered_set<int> satisfiedTasksIds);

    bool removeTasksAndWPs(const std::unordered_set<int> &rmvTasksIndices);
    TimeStep getTTD() const;
    TimeStep getlastDeliveryTimeStep() const;
    const WaypointsList& getWaypoints() const;
    const Path& getPath() const;
    const std::unordered_set<int>& getSatisfiedTasksIds() const;

    CompressedCoord getInitialPos() const;
private:
    Path path;
    WaypointsList wpList;
    std::unordered_set<int> satisfiedTasksIds;
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

#endif //SIMULTANEOUS_CMAPD_PATHWRAPPER_HPP
