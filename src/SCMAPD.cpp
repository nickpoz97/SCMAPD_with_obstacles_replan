#include <algorithm>
#include <boost/iterator/counting_iterator.hpp>
#include "SCMAPD.hpp"
#include "Assignment.hpp"

SCMAPD::SCMAPD(
        cmapd::AmbientMapInstance&& ambientMapInstance,
        std::vector<Assignment> &&robots,
        std::vector<Task> && tasksVector
    ) :
    status(std::move(ambientMapInstance), std::move(robots), std::move(tasksVector)),
    bigH(buildPartialAssignmentHeap(status))
    {}

BigH
SCMAPD::buildPartialAssignmentHeap(const Status &status) {
    BigH totalHeap{};

    for(int ti = 0 ; ti < status.getTasks().size() ; ++ti){
        std::vector<Assignment> pa;
        const auto& robots = status.getAssignments();
        pa.reserve(robots.size());

        // add "task" to each robot
        for (const auto& r : robots){
            // robot in partial assignments heap
            pa.emplace_back(
                initializePartialAssignment(status, ti, r)
            );
        }
        std::sort(
            pa.begin(),
            pa.end()
        );
        totalHeap.push_back({ti, std::move(pa)});
    }
    totalHeap.sort(compareSmallH);
    return totalHeap;
}

Assignment
SCMAPD::initializePartialAssignment(const Status &status, int taskIndex, const Assignment &robot) {
    Assignment robotCopy{Assignment{robot}};
    const auto& task = status.getTask(taskIndex);

    robotCopy.setTasks(
            {{task.startLoc, Demand::START, task.index},
             {task.goalLoc,  Demand::GOAL,  task.index}},
            status.getTasks());
    return robotCopy;
}

void SCMAPD::solve(Heuristic heuristic, TimeStep cutOffTime) {
    // extractTop takes care of tasks indices removal
    for(auto candidateAssignment = extractTop(); !status.getUnassignedTasksIndices().empty(); candidateAssignment = extractTop()){
        auto robotIndex = candidateAssignment.getIndex();
        // todo check this
        status.getAssignment(robotIndex) = std::move(candidateAssignment);

        for (auto& [taskId, partialAssignments] : bigH){
            auto& pa = partialAssignments[robotIndex];
            pa.insert(taskId, status, heuristic);
            updateSmallHTop(robotIndex, 0, partialAssignments);
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
    status.removeTaskIndex(taskId);

    return candidateAssignment;
}

void SCMAPD::updateSmallHTop(int assignmentIndex, int v, std::vector<Assignment> &partialAssignments) {
    const Assignment& a = status.getAssignment(assignmentIndex);

    for (int i = 0 ; i < v ; ++i) {
        auto& targetPA = partialAssignments[i];
        if(Assignment::hasConflicts(a, targetPA)) {
            targetPA.recomputePath(a.getConstraints(), status);
            // restart
            if(v > 0){
                Assignment& minPA = *std::min_element(partialAssignments.begin(), partialAssignments.begin() + v);
                Assignment& firstPA = *partialAssignments.begin();
                std::swap(minPA, firstPA);
                i = 0;
            }
        }
    }
}
