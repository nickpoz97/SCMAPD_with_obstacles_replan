#include <filesystem>
#include <boost/iterator/counting_iterator.hpp>
#include <SCMAPD.hpp>

SCMAPD::SCMAPD(
        DistanceMatrix && loadedDistanceMatrix,
        Assignment &&robots,
        TasksVector && tasksVector
    ) :
        distanceMatrix(std::move(loadedDistanceMatrix)),
        assignment(std::move(robots)),
        tasks(std::move(tasksVector)),
        unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasks.size())),
        partialAssignmentsHeap(buildPartialAssignmentHeap(assignment, tasks, distanceMatrix))
    {}

PartialAssignmentHeap
SCMAPD::buildPartialAssignmentHeap(const Assignment &robots, const TasksVector &tasks,
                                   const DistanceMatrix &distanceMatrix) {
    PartialAssignmentHeap partialAssignmentsHeap{};

    for(const auto& task : tasks){
        // partial assignment for "task"
        PartialAssignment partialAssignment;
        partialAssignment.reserve(robots.size());

        // add "task" to each robot
        for (const auto& robotKV : robots){
            auto robotStartPos = robotKV.first;

            // robot in partial assignment heap
            Robot robotCopy{robotKV.second};

            // no conflicts at the beginning (simplified expression)
            auto ttd =
                distanceMatrix[robotStartPos][task.startLoc] -
                task.releaseTime;

            robotCopy.setTasksAndTTD({task.startLoc, task.goalLoc}, ttd);

            partialAssignment.push_back(std::move(robotCopy));
        }
        partialAssignmentsHeap.push(std::move(partialAssignment));
    }

    return partialAssignmentsHeap;
}

bool ComparePartialAssignment::operator()(const Robot& a, const Robot& b) {
    return a.getTtd() > b.getTtd();
}

bool CompareTotalHeap::operator()(const PartialAssignment &a, const PartialAssignment &b) {
    return a[0].getTtd() > b[0].getTtd();
}

template<Heuristic heuristic>
void SCMAPD::solve(TimeStep cutOffTime) {
    while(!tasks.empty()){
        // top() refers to tasks, [0] to Robot (and so waypoints)
        const Robot& partialAssignment = partialAssignmentsHeap.top()[0];

        // todo continue

    }
}

template<>
Waypoints SCMAPD::insert<Heuristic::MCA>(const Task &task, const Waypoints &waypoints) {
    // todo complete this
    return {};
}

template Waypoints SCMAPD::insert<Heuristic::HEUR>(const Task &task, const Waypoints &waypoints);
template void SCMAPD::solve<Heuristic::HEUR>(TimeStep cutOffTime);
