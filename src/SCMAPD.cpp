#include <filesystem>
#include "SCMAPD.hpp"

template<Heuristic heuristic>
SCMAPD<heuristic>::SCMAPD(
        DistanceMatrix && distanceMatrix,
        Assignment && robots,
        std::unordered_set<Task> && tasks
    ) :
    distanceMatrix(std::move(distanceMatrix)),
    assignment(std::move(robots)),
    unassignedTasks(std::move(tasks)),
    partialAssignmentsHeap(buildPartialAssignmentHeap(robots, tasks, distanceMatrix))
    {}

template<>
Waypoints SCMAPD<Heuristic::MCA>::insert(const Task &task, const Waypoints &waypoints) {
    // todo complete this
    return {};
}

template<Heuristic heuristic>
void SCMAPD<heuristic>::solve(const PBS &pbs) {

}

template<Heuristic heuristic>
PartialAssignmentHeap
SCMAPD<heuristic>::buildPartialAssignmentHeap(const Assignment &robots, const std::unordered_set<Task> &tasks,
                                              const DistanceMatrix &distanceMatrix) {
    PartialAssignmentHeap partialAssignmentsHeap{};

    for(const auto& task : tasks){
        Assignment partialAssignment{robots};
        for (auto& robot : partialAssignment){
            // no conflicts at the beginning (simplified expression)
            auto ttd =
                distanceMatrix[robot.getStart()][task.getStartLoc()] -
                task.getReleaseTime();
            robot.setTasksAndTTD({task.getStartLoc(), task.getGoalLoc()}, ttd);
        }
        partialAssignmentsHeap.push(std::move(partialAssignment));
    }

    return partialAssignmentsHeap;
}

bool ComparePartialAssignment::operator()(const Robot& a, const Robot& b) {
    return a.getTtd() > b.getTtd();
}

bool CompareTotalHeap::operator()(const Assignment &a, const Assignment &b) {
    return a[0].getTtd() > b[0].getTtd();
}
