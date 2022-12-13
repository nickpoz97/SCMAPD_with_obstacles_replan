#include <algorithm>
#include "SCMAPD.hpp"
#include "Assignment.hpp"

SCMAPD::SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
               std::vector<Task> &&tasksVector, Heuristic heuristic) :
    status(std::move(ambientMapInstance), std::move(robots), std::move(tasksVector)),
    heuristic(heuristic),
    bigH(buildPartialAssignmentHeap(status, heuristic))
    {
        status.print();
    }

BigH
SCMAPD::buildPartialAssignmentHeap(const Status &status, Heuristic heuristic) {
    BigH totalHeap{};

    for(int ti = 0 ; ti < status.getTasks().size() ; ++ti){
        std::vector<Assignment> partialAssignments;
        const auto& robots = status.getAssignments();
        partialAssignments.reserve(robots.size());

        // add "task" to each robot
        for (const auto& r : robots){
            // robot in partial assignments heap
            partialAssignments.emplace_back(
                initializePartialAssignment(status, ti, r)
            );
        }
        std::sort(
            partialAssignments.begin(),
            partialAssignments.end()
        );
        totalHeap.push_back({ti, std::move(partialAssignments)});
    }
    sortBigH(totalHeap, heuristic);
    return totalHeap;
}

Assignment
SCMAPD::initializePartialAssignment(const Status &status, int taskIndex, const Assignment &robot) {
    Assignment robotCopy{Assignment{robot}};
    const auto& task = status.getTask(taskIndex);
    auto k = robot.getIndex();

    robotCopy.insert(
        task.index,
        status.getAmbientMapInstance(),
        status.getTasks(),
        {}
    );
    return robotCopy;
}

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractTop takes care of tasks indices removal
    for(auto [taskId, candidateAssignment] = extractTop();
        !status.getUnassignedTasksIndices().empty();
        std::tie(taskId, candidateAssignment) = extractTop()
    ){
        auto k = candidateAssignment.getIndex();

        auto& assignment = status.getAssignment(k);
        assignment = std::move(candidateAssignment);

        for (auto& [otherTaskId, partialAssignments] : bigH){
            auto& pa = *findPA(partialAssignments, k);
            pa.insert(taskId, status.getAmbientMapInstance(), status.getTasks(), status.getOtherConstraints(k));
            updateSmallHTop(
                assignment,
                heuristic == Heuristic::MCA ? 1 : 2,
                partialAssignments
            );
        }
        sortBigH(bigH, heuristic);
    }
    status.print();
}

std::pair<int, Assignment> SCMAPD::extractTop() {
    // top() refers to tasks, [0] to PartialAssignment (and so waypoints) (pair<int, ptr>)
    // thanks to shared pointer, the heap does not destroy the object and partialAssignmentsPtr doesn't throw SIGSEG
    auto& [taskId, partialAssignments] = bigH.front();
    auto candidateAssignment{std::move(partialAssignments.at(0))};

#ifndef NDEBUG
    auto oldRemainingTasks = status.getUnassignedTasksIndices().size();
    auto oldBigHSIze = bigH.size();
#endif

    bigH.pop_front();
    status.removeTaskIndex(taskId);

    assert(oldBigHSIze == bigH.size() + 1 && oldRemainingTasks == status.getUnassignedTasksIndices().size() + 1);

    return {taskId, candidateAssignment};
}

void SCMAPD::updateSmallHTop(const Assignment &fixedAssignment, int v, std::vector<Assignment> &partialAssignments) {
    std::sort(partialAssignments.begin(), partialAssignments.end());

    for (int k = 0 ; k < v ; ++k) {
        auto& targetPA = *findPA(partialAssignments, k);
        if(Assignment::hasConflicts(fixedAssignment, targetPA)) {
            targetPA.internalUpdate(status.getOtherConstraints(k), status.getTasks(), status.getAmbientMapInstance());
            // todo min search on first v elements or everyone?
            std::sort(partialAssignments.begin(), partialAssignments.end());
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

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                       const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                       Heuristic heuristic) {
    DistanceMatrix dm(cnpy::npy_load(distanceMatrixFile));

    auto robots{loadAssignments(agentsFile, dm.nCols)};
    auto tasks{loadTasks(tasksFile, dm.nCols)};


    cmapd::AmbientMapInstance instance(
            cmapd::AmbientMap(gridFile, dm.nRows, dm.nCols),
            {robots.begin(), robots.end()},
            {tasks.begin(), tasks.end()},
            std::move(dm)
    );

#ifndef NDEBUG
    assert(instance.agents().size() == robots.size());
    for (int i = 0 ; i < instance.agents().size() ; ++i){
        assert(instance.agents()[i] == static_cast<Coord>(robots[i]));
    }
    assert(instance.tasks().size() == tasks.size());
    for (int i = 0 ; i < instance.tasks().size() ; ++i){
        const auto& t = static_cast<std::pair<Coord,Coord>>(tasks[i]);
        assert(instance.tasks()[i] == t);
    }
#endif

    return {std::move(instance), std::move(robots), std::move(tasks), heuristic};
}
