#include <filesystem>
#include <boost/iterator/counting_iterator.hpp>
#include <SCMAPD.hpp>

SCMAPD::SCMAPD(
        DistanceMatrix && loadedDistanceMatrix,
        RobotsVector &&robots,
        TasksVector && tasksVector
    ) :
        distanceMatrix(std::move(loadedDistanceMatrix)),
        assignments(std::move(robots)),
        tasks(std::move(tasksVector)),
        unassignedTasksIndices(boost::counting_iterator<int>(0), boost::counting_iterator<int>(tasks.size())),
        totalHeap(buildPartialAssignmentHeap(assignments, tasks, distanceMatrix))
    {}

TotalHeap
SCMAPD::buildPartialAssignmentHeap(const RobotsVector &robots, const TasksVector &tasks,
                                   const DistanceMatrix &distanceMatrix) {
    TotalHeap totalHeap{};

    for(const auto& task : tasks){
        std::shared_ptr<RobotsVector> partialAssignmentsPtr{new RobotsVector {}};

        partialAssignmentsPtr->reserve(robots.size());

        // add "task" to each robot
        for (const Robot& robot : robots){
            // robot in partial assignments heap
            partialAssignmentsPtr->push_back(
                initializePartialAssignment(distanceMatrix, task, robot)
            );
        }
        totalHeap.emplace(task.index, partialAssignmentsPtr);
    }

    return totalHeap;
}

Robot
SCMAPD::initializePartialAssignment(const DistanceMatrix &distanceMatrix, const Task &task, const Robot &robot) {
    Robot robotCopy{robot};

    // no conflicts at the beginning (simplified expression)
    auto ttd =
        distanceMatrix[robot.getStartPosition()][task.startLoc] -
        task.releaseTime;

    robotCopy.setTasksAndTTD({task.startLoc, task.goalLoc}, ttd);
    return robotCopy;
}

bool ComparePartialAssignment::operator()(const Robot & a, const Robot & b) {
    return a.getTtd() > b.getTtd();
}

template<Heuristic heuristic>
void SCMAPD::solve(TimeStep cutOffTime) {
    for(auto candidateAssignmentsPtr = extractTop().second; !unassignedTasksIndices.empty(); candidateAssignmentsPtr = extractTop().second){
        Robot& candidateAssignment = candidateAssignmentsPtr->at(0);
        assignments[candidateAssignment.getIndex()].setTasksAndTTD(candidateAssignment);

        for (auto [taskId, partialAssignmentPointer] : totalHeap){

        }
        // todo insert waypoints and update heaps
    }
}

template<>
Waypoints SCMAPD::insert<Heuristic::MCA>(const Task &task, const Waypoints &waypoints) {
    // todo complete this
    return {};
}

PartialAssignmentsVector SCMAPD::extractTop() {
    // top() refers to tasks, [0] to Robot (and so waypoints) (pair<unsigned, ptr>)
    // thanks to shared pointer, the heap does not destroy the object and partialAssignmentsPtr doesn' t throw SIGSEG
    auto top{totalHeap.top()};
    totalHeap.pop();
    unassignedTasksIndices.erase(top.first);

    return top;
}

template Waypoints SCMAPD::insert<Heuristic::HEUR>(const Task &task, const Waypoints &waypoints);
template void SCMAPD::solve<Heuristic::HEUR>(TimeStep cutOffTime);

bool CompareTotalHeap::operator()(const PartialAssignmentsVector & a, const PartialAssignmentsVector & b) {
    return a.second->at(0).getTtd() > b.second->at(0).getTtd();
}
