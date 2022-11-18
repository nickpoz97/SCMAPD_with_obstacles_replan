#include "PartialAssignment.hpp"

template<Heuristic heuristic>
unsigned PartialAssignment<heuristic>::getTaskIndex() const {
    return taskIndex;
}

template<Heuristic heuristic>
TimeStep PartialAssignment<heuristic>::getTtd() const {
    return robot.getTtd();
}

template<Heuristic heuristic>
Waypoints PartialAssignment<heuristic>::releaseWaypoints() {
    return robot.releaseWaypoints();
}

// todo initialize depending on emptiness
template<Heuristic heuristic>
PartialAssignment<heuristic>::PartialAssignment(const Robot &robot, const Task &task, const DistanceMatrix& distanceMatrix) :
    robot(insert(robot, task, distanceMatrix)),
    taskIndex{task.index}
    {}

template<Heuristic heuristic>
Robot PartialAssignment<heuristic>::insert(const Robot &robot, const Task &task, const DistanceMatrix& distanceMatrix) {
    return robot.empty() ? initialize(robot, task, distanceMatrix) : update(robot, task, distanceMatrix);
}

template<Heuristic heuristic>
Robot PartialAssignment<heuristic>::initialize(const Robot &robot, const Task &task, const DistanceMatrix &distanceMatrix) {
    Robot robotCopy{robot};

    // no conflicts at the beginning (simplified expression)
    auto ttd =
            distanceMatrix[robot.getPosition()][task.startLoc] -
            task.releaseTime;

    robotCopy.setTasksAndTTD({task.startLoc, task.goalLoc}, ttd);

    return robotCopy;
}

template<Heuristic heuristic>
Robot PartialAssignment<heuristic>::update(const Robot &robot, const Task &task, const DistanceMatrix &vector) {
    // todo complete alg
    return Robot(0, 0, 0);
}
