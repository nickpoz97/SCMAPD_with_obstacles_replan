#ifndef SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP

#include <forward_list>
#include "Robot.hpp"

class PartialAssignment : public Robot{
public:
    void setTasksAndTTD(Waypoints &&newWaypoints, TimeStep newTtd,  std::forward_list<unsigned>&& newTaskIndices);
    void setTasksAndTTD(const Waypoints &newWaypoints, TimeStep newTtd, const std::forward_list<unsigned>& newTaskIndices);
    void insert(const Task& task);
private:
    // index of task associated to each waypoint
    std::list<unsigned> demands;

    void insertTaskWaypoints(const Task &task, std::list<CompressedCoord>::iterator &waypointStart,
                             std::list<CompressedCoord>::iterator &waypointGoal,
                             std::list<unsigned int>::iterator &demandsStart,
                             std::list<unsigned int>::iterator &demandsGoal);

    bool checkCapacityConstraint();

    void restorePreviousWaypoints(std::list<CompressedCoord>::iterator &waypointStart,
                                  std::list<CompressedCoord>::iterator &waypointGoal,
                                  std::list<unsigned int>::iterator &demandsStart,
                                  std::list<unsigned int>::iterator &demandsGoal);
};


#endif //SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
