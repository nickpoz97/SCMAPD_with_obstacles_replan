/// @file Assignment.hpp

#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <list>
#include <optional>

#include "Task.hpp"
#include "NewTypes.hpp"
#include "Waypoint.hpp"
#include "Status.hpp"
#include "AgentInfo.hpp"
#include "PathWrapper.hpp"

/**
 * @class Assignment
 * @brief class that abstracts an agent and its path with other infos
 */
class Assignment : public PathWrapper{

public:
    /**
     *
     * @param startPosition initial position of the agent
     * @param index numerical id for the agent
     * @param capacity max number of tasks the agent can keep
     */
    explicit Assignment(const AgentInfo &agentInfo);

    /// @return difference between actual TTD and TTD before last task addition to path
    [[nodiscard]] TimeStep getMCA() const;

    /// @return agent numerical id
    [[nodiscard]] int getAgentId() const;

    /// @return true if agent contains no waypoints
    [[nodiscard]] bool empty() const;

    /**
     * @brief add a new task in the best position and recompute path
     * @param taskId
     * @param ambientMapInstance
     * @param tasks
     * @param constraints
     */
    [[nodiscard]] bool addTask(int taskId, const Status &status);

//    friend bool operator>(const Assignment &a, const Assignment &b);
//    friend bool operator<(const Assignment &a, const Assignment &b);
    friend int operator<=>(const Assignment &a, const Assignment &b);

    // this should be called when waypoints and/or constraints are changed
    [[nodiscard]] bool internalUpdate(const Status &status, bool updateMCA);

    Assignment& operator=(const PathWrapper& otherPW);
private:
    int index;
    int mca = 0;

    TimeStep oldTTD = 0;
};

#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
