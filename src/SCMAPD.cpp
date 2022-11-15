#include <filesystem>
#include "SCMAPD.hpp"


template<Heuristic heuristic>
SCMAPD<heuristic>::SCMAPD(
        DistanceMatrix && distanceMatrix,
        Assignment && robots,
        std::unordered_set<Task> && tasks,
        PBS&& pbs
    ) :
    distanceMatrix(std::move(distanceMatrix)),
    assignment(std::move(robots)),
    unassignedTasks(std::move(tasks)),
    pbs{std::move(pbs)}
    {}

template<>
Waypoints SCMAPD<Heuristic::MCA>::insert(const Task &task, const Waypoints &waypoints) {
    // todo complete this
    return {};
}

template<Heuristic heuristic>
void SCMAPD<heuristic>::solve() {

}

template<Heuristic heuristic>
SCMAPD<heuristic>::~SCMAPD() {
    pbs.clearSearchEngines();
}

template<Heuristic heuristic>
PartialAssignmentHeap
SCMAPD<heuristic>::buildPartialAssignmentHeap(const Assignment &robots, const std::unordered_set<Task> &tasks) {
    for(const auto& task : tasks){
        Assignment partialAssignment{robots};
        for (auto& i : partialAssignment){
            i.setTasksAndTTD({task.getStartLoc(), task.getGoalLoc()}, 0);
        }
    }
}

bool ComparePartialAssignment::operator()(const Robot& a, const Robot& b) {
    return !(a.getTtd() <= b.getTtd()) ;
}

bool CompareTotalHeap::operator()(const Assignment &a, const Assignment &b) {
    return !(a[0].getTtd() <= b[0].getTtd());
}
