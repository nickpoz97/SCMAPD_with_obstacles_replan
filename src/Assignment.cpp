#include "Assignment.hpp"
#include <algorithm>

Assignment::Assignment(const std::vector<Robot> &robots) :
    assignmentsPerRobot(initializeAssignments(robots))
    {}

AssignmentsPerRobot Assignment::initializeAssignments(const std::vector<Robot> &robots) {
    AssignmentsPerRobot assignmentPerRobot(robots.size());

    std::transform(
        robots.cbegin(),
        robots.cend(),
        assignmentPerRobot.begin(),
        [](const Robot& r){return SequenceOfActions{r.getStart()}; }
    );

    return assignmentPerRobot;
}
