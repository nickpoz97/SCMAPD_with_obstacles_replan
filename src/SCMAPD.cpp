#include <algorithm>
#include "SCMAPD.hpp"
#include "Assignment.hpp"

SCMAPD::SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
               std::vector<Task> &&tasksVector, Heuristic heuristic) :
    status(std::move(ambientMapInstance), std::move(robots), std::move(tasksVector)),
    heuristic(heuristic),
    bigH(buildPartialAssignmentHeap(status, heuristic))
    {}

BigH
SCMAPD::buildPartialAssignmentHeap(const Status &status, Heuristic heuristic) {
    BigH totalHeap{};

    for(int ti = 0 ; ti < status.getTasks().size() ; ++ti){
        std::vector<Assignment> pas;
        const auto& robots = status.getAssignments();
        pas.reserve(robots.size());

        // add "task" to each robot
        for (const auto& r : robots){
            // robot in partial assignments heap
            pas.emplace_back(
                initializePartialAssignment(status, ti, r)
            );
        }
        std::sort(
            pas.begin(),
            pas.end()
        );
        totalHeap.push_back({ti, std::move(pas)});
    }
    sortBigH(totalHeap, heuristic);
    return totalHeap;
}

Assignment
SCMAPD::initializePartialAssignment(const Status &status, int taskIndex, const Assignment &robot) {
    Assignment robotCopy{Assignment{robot}};
    const auto& task = status.getTask(taskIndex);
    auto k = robot.getIndex();

    robotCopy.setTasks(
            {{task.startLoc, Demand::START, task.index},
             {task.goalLoc,  Demand::GOAL,  task.index}},
            status.getOtherConstraints(k), status.getAmbientMapInstance(), status.getTasks());
    return robotCopy;
}

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractTop takes care of tasks indices removal
    for(auto candidateAssignment = extractTop(); !status.getUnassignedTasksIndices().empty(); candidateAssignment = extractTop()){
        auto k = candidateAssignment.getIndex();
        // todo check this
        status.getAssignment(k) = std::move(candidateAssignment);

        for (auto& [taskId, partialAssignments] : bigH){
            auto& pa = partialAssignments[k];
            pa.insert(taskId, status.getAmbientMapInstance(), status.getTasks(), status.getOtherConstraints(k));
            updateSmallHTop(k, heuristic == Heuristic::MCA ? 1 : 2, partialAssignments);
        }
        sortBigH(bigH, heuristic);
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

    for (int k = 0 ; k < v ; ++k) {
        auto& targetPA = partialAssignments[k];
        if(Assignment::hasConflicts(a, targetPA)) {
            targetPA.internalUpdate(status.getOtherConstraints(k), status.getTasks(), status.getAmbientMapInstance());
            // todo min search on first v elements or everyone?
            Assignment &minPA = *std::min_element(partialAssignments.begin(), partialAssignments.begin() + v);
            Assignment &firstPA = *partialAssignments.begin();
            std::swap(minPA, firstPA);
            // restart
            k = 0;
        }
    }
}

void SCMAPD::sortBigH(BigH &bigH, Heuristic heuristic) {
    switch(heuristic){
        case Heuristic::MCA:
            bigH.sort(
                [](const SmallH& a, const SmallH& b){return a.second[0] < b.second[0];}
            );
            break;
        case Heuristic::RMCA_A:
            bigH.sort(
                [](const SmallH& a, const SmallH& b) {
                    auto aVal = a.second[0].getMCA() - a.second[1].getMCA();
                    auto bVal = b.second[0].getMCA() - b.second[1].getMCA();

                    return aVal > bVal;
                }
            );
            break;
        case Heuristic::RMCA_R:
            bigH.sort(
                [](const SmallH& a, const SmallH& b) {
                    auto aVal = a.second[0].getMCA() / a.second[1].getMCA();
                    auto bVal = b.second[0].getMCA() / b.second[1].getMCA();

                    return aVal > bVal;
                }
            );
            break;
    }
}
