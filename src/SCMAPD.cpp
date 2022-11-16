#include <filesystem>
#include <SCMAPD.hpp>

SCMAPD::SCMAPD(
        DistanceMatrix && distanceMatrix,
        Assignment && robots,
        TaskSet && tasks
    ) :
    distanceMatrix(std::move(distanceMatrix)),
    assignment(std::move(robots)),
    unassignedTasks(std::move(tasks)),
    partialAssignmentsHeap(buildPartialAssignmentHeap(robots, tasks, distanceMatrix))
    {}

PartialAssignmentHeap
SCMAPD::buildPartialAssignmentHeap(const Assignment &robots, const TaskSet &tasks,
                                              const DistanceMatrix &distanceMatrix) {
    PartialAssignmentHeap partialAssignmentsHeap{};

    for(const auto& task : tasks){
        Assignment partialAssignment{robots};
        for (auto& robot : partialAssignment){
            // no conflicts at the beginning (simplified expression)
            auto ttd =
                distanceMatrix[robot.getStart()][task.startLoc] -
                task.releaseTime;
            robot.setTasksAndTTD({task.startLoc, task.goalLoc}, ttd);
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

template<>
void SCMAPD::solve<Heuristic::MCA>(const PBS &pbs) {
    // todo complete this
}

template<>
Waypoints SCMAPD::insert<Heuristic::MCA>(const Task &task, const Waypoints &waypoints) {
    // todo complete this
    return {};
}
