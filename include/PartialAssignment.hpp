#ifndef SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP

#include <forward_list>
#include "Robot.hpp"

class PartialAssignment : public Robot{
public:
    const std::forward_list<unsigned int> &getTasksIndices() const;
    void setTasksAndTTD(Waypoints &&newWaypoints, TimeStep newTtd,  std::forward_list<unsigned>&& newTaskIndices);
    void setTasksAndTTD(const Waypoints &newWaypoints, TimeStep newTtd, const std::forward_list<unsigned>& newTaskIndices);
private:
    // index of task associated to each waypoint
    std::forward_list<unsigned> tasksIndices;
};


#endif //SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
