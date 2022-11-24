#include <filesystem>
#include <boost/iterator/counting_iterator.hpp>
#include <SCMAPD.hpp>

SCMAPD::SCMAPD(
        DistanceMatrix && loadedDistanceMatrix,
        std::vector<Robot> &&robots,
        TasksVector && tasksVector
    ) :
        distanceMatrix(std::move(loadedDistanceMatrix)),
        assignments(std::move(robots)),
        tasks(std::move(tasksVector)),
        unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasks.size())),
        totalHeap(buildPartialAssignmentHeap(assignments, tasks, distanceMatrix))
    {}

TotalHeap
SCMAPD::buildPartialAssignmentHeap(const std::vector<Robot> &robots, const TasksVector &tasks,
                                   const DistanceMatrix &distanceMatrix) {
    TotalHeap totalHeap{};

    for(const auto& task : tasks){
        std::vector<RobotSmartPtr> pa;
        pa.reserve(robots.size());

        // add "task" to each robot
        for (const Robot& robot : robots){
            // robot in partial assignments heap
            pa.emplace_back(
                initializePartialAssignment(distanceMatrix, task, robot)
            );
        }
        std::sort(
            pa.begin(),
            pa.end(),
            comparePartialAssignment
        );
        totalHeap.push_back({task.index, std::move(pa)});
    }
    totalHeap.sort(compareTotalHeap);
    return totalHeap;
}

RobotSmartPtr
SCMAPD::initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Robot &robot) {
    RobotSmartPtr robotCopyPtr{new Robot{robot}};

    // no conflicts at the beginning (simplified expression)
    auto ttd =
        distanceMatrix[robot.getStartPosition()][task.startLoc] -
        task.releaseTime;

    robotCopyPtr->setTasksAndTTD(
        {{task.startLoc, Demand::START} , {task.goalLoc, Demand::GOAL}},
        ttd
    );
    return robotCopyPtr;
}

template<Heuristic heuristic>
void SCMAPD::solve(TimeStep cutOffTime) {
    // extractTop takes care of tasks indices removal
    for(auto candidateAssignmentPtr = extractTop(); !unassignedTasksIndices.empty(); candidateAssignmentPtr = extractTop()){
        auto robotIndex = candidateAssignmentPtr->getIndex();
        assignments[robotIndex].setTasksAndTTD(*candidateAssignmentPtr);

        for (auto& [taskId, partialAssignments] : totalHeap){
            partialAssignments[robotIndex]->insert(tasks[taskId]);
            updatePAsHeapTop(partialAssignments);
        }
        // should be same complexity as using priority_queue
        totalHeap.sort(compareTotalHeap);
    }
}

RobotSmartPtr SCMAPD::extractTop() {
    // top() refers to tasks, [0] to Robot (and so waypoints) (pair<unsigned, ptr>)
    // thanks to shared pointer, the heap does not destroy the object and partialAssignmentsPtr doesn't throw SIGSEG
    auto& [taskId, partialAssignments] = totalHeap.front();
    RobotSmartPtr candidateAssignment{partialAssignments.at(0).release()};

    totalHeap.pop_front();
    unassignedTasksIndices.erase(taskId);

    return candidateAssignment;
}

void SCMAPD::updatePAsHeapTop(std::vector<RobotSmartPtr>& partialAssignments) {
    // todo complete this
}

template void SCMAPD::solve<Heuristic::HEUR>(TimeStep cutOffTime);
