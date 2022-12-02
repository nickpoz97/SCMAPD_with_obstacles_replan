#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include "SCMAPD.hpp"
#include "Assignment.hpp"

SCMAPD::SCMAPD(
        cmapd::AmbientMapInstance&& ambientMapInstance,
        std::vector<Assignment> &&robots,
        std::vector<Task> && tasksVector
    ) :
        ambientMapInstance(std::move(ambientMapInstance)),
        assignments(std::move(robots)),
        tasks(tasksVector),
        unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasks.size())),
        bigH(buildPartialAssignmentHeap(assignments, tasks, ambientMapInstance.h_table()))
    {}

BigH
SCMAPD::buildPartialAssignmentHeap(const std::vector<Assignment> &robots, const std::vector<Task> &tasks,
                                   const DistanceMatrix &distanceMatrix) {
    BigH totalHeap{};

    for(const auto& t : tasks){
        std::vector<Assignment> pa;
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
            pa.end()
        );
        totalHeap.push_back({t.index, std::move(pa)});
    }
    totalHeap.sort(compareSmallH);
    return totalHeap;
}

Assignment
SCMAPD::initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Assignment &robot,
                                    const std::vector<Task> &taskVector) {
    Assignment robotCopy{Assignment{robot}};

    robotCopy.setTasks(
            {{task.startLoc, Demand::START, task.index},
             {task.goalLoc,  Demand::GOAL,  task.index}},
            taskVector, distanceMatrix);
    return robotCopy;
}

void SCMAPD::solve(Heuristic heuristic, TimeStep cutOffTime) {
    // extractTop takes care of tasks indices removal
    for(auto candidateAssignment = extractTop(); !unassignedTasksIndices.empty(); candidateAssignment = extractTop()){
        auto robotIndex = candidateAssignment.getIndex();
        assignments[robotIndex].update(std::move(candidateAssignment));

        for (auto& [taskId, partialAssignments] : bigH){
            partialAssignments[robotIndex].insert(tasks[taskId], distanceMatrix, tasks, heuristic);
            updateSmallHTop(partialAssignments);
        }
        bigH.sort(compareSmallH);
    }
}

Assignment SCMAPD::extractTop() {
    // top() refers to tasks, [0] to PartialAssignment (and so waypoints) (pair<unsigned, ptr>)
    // thanks to shared pointer, the heap does not destroy the object and partialAssignmentsPtr doesn't throw SIGSEG
    auto& [taskId, partialAssignments] = bigH.front();
    auto candidateAssignment{std::move(partialAssignments.at(0))};

    bigH.pop_front();
    unassignedTasksIndices.erase(taskId);

    return candidateAssignment;
}

void SCMAPD::updateSmallHTop(const std::vector<Assignment> &partialAssignments) {
    // todo complete this
}
