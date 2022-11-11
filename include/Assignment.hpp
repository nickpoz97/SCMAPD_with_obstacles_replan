#ifndef SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP

#include <vector>
#include <forward_list>
#include <Robot.hpp>
#include <TypeDefs.hpp>

class Assignment {
public:
    Assignment(const std::vector<Robot>& robots);
private:
    AssignmentsPerRobot assignmentsPerRobot;

    static AssignmentsPerRobot initializeAssignments(const std::vector<Robot>& robots);
};


#endif //SIMULTANEOUS_CMAPD_ASSIGNMENT_HPP
