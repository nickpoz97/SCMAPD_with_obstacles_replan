#include <filesystem>
#include <boost/iterator/counting_iterator.hpp>
#include <SCMAPD.hpp>
#include "Assignment.hpp"

SCMAPD::SCMAPD(
        DistanceMatrix && loadedDistanceMatrix,
        std::vector<Assignment> &&robots,
        TasksVector && tasksVector
    ) :
        distanceMatrix(std::move(loadedDistanceMatrix)),
        assignments(std::move(robots)),
        tasks(std::move(tasksVector)),
        unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasks.size())),
        totalHeap(buildPartialAssignmentHeap(assignments, tasks, distanceMatrix))
    {}

TotalHeap
SCMAPD::buildPartialAssignmentHeap(const std::vector<Assignment> &robots, const TasksVector &tasks,
                                   const DistanceMatrix &distanceMatrix) {
    TotalHeap totalHeap{};

    for(const auto& t : tasks){
        std::vector<PASmartPtr> pa;
        pa.reserve(robots.size());

        // add "task" to each robot
        for (const auto& robot : robots){
            // robot in partial assignments heap
            pa.emplace_back(
                    initializePartialAssignment(distanceMatrix, t, robot, tasks)
            );
        }
        std::sort(
            pa.begin(),
            pa.end(),
            comparePartialAssignment
        );
        totalHeap.push_back({t.index, std::move(pa)});
    }
    totalHeap.sort(compareTotalHeap);
    return totalHeap;
}

PASmartPtr
SCMAPD::initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Assignment &robot,
                                    const TasksVector &taskVector) {
    PASmartPtr robotCopyPtr{new Assignment{robot}};

    robotCopyPtr->setTasks(
            {{task.startLoc, Demand::START, task.index},
             {task.goalLoc,  Demand::GOAL,  task.index}},
            taskVector, distanceMatrix);
    return robotCopyPtr;
}

void SCMAPD::solve(Heuristic heuristic, TimeStep cutOffTime) {
    // extractTop takes care of tasks indices removal
    for(auto candidateAssignmentPtr = extractTop(); !unassignedTasksIndices.empty(); candidateAssignmentPtr = extractTop()){
        auto robotIndex = candidateAssignmentPtr->getIndex();
        assignments[robotIndex].setTasks(*candidateAssignmentPtr, tasks, distanceMatrix);

        for (auto& [taskId, partialAssignments] : totalHeap){
            partialAssignments[robotIndex]->insert(tasks[taskId], heuristic, distanceMatrix, tasks);
            updatePAsHeapTop(partialAssignments);
        }
        // should be same complexity as using priority_queue
        totalHeap.sort(compareTotalHeap);
    }
}

PASmartPtr SCMAPD::extractTop() {
    // top() refers to tasks, [0] to PartialAssignment (and so waypoints) (pair<unsigned, ptr>)
    // thanks to shared pointer, the heap does not destroy the object and partialAssignmentsPtr doesn't throw SIGSEG
    auto& [taskId, partialAssignments] = totalHeap.front();
    PASmartPtr candidateAssignment{partialAssignments.at(0).release()};

    totalHeap.pop_front();
    unassignedTasksIndices.erase(taskId);

    return candidateAssignment;
}

void SCMAPD::updatePAsHeapTop(std::vector<PASmartPtr>& partialAssignments) {
    // todo complete this
}
