#include <algorithm>
#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"

SCMAPD::SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
               std::vector<Task> &&tasksVector, Heuristic heuristic, bool debug) :
    status(std::move(ambientMapInstance), std::move(robots), std::move(tasksVector)),
    heuristic(heuristic),
    bigH(buildPartialAssignmentHeap(status, heuristic)),
    debug{debug}
    {
    if(debug)
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
        sortPA(partialAssignments);
        totalHeap.push_back({ti, std::move(partialAssignments)});
    }
    sortBigH(totalHeap, heuristic);
    return totalHeap;
}

Assignment
SCMAPD::initializePartialAssignment(const Status &status, int taskIndex, const Assignment &robot) {
    Assignment robotCopy{Assignment{robot}};

    robotCopy.addTask(
            status.getAmbientMapInstance(),
            {}, <#initializer#>);
    return robotCopy;
}

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractBigHTop takes care of tasks indices removal
    while( !status.getUnassignedTasksIndices().empty() ){
        assert(!status.checkCollisions());
        auto [taskId, candidateAssignment] = extractBigHTop();
        auto k = status.update(std::move(candidateAssignment));

        for (auto& [otherTaskId, partialAssignments] : bigH){
            auto& pa = *findPA(partialAssignments, k);
            pa.addTask(status.getAmbientMapInstance(), status.getConstraints(), <#initializer#>);
            updateSmallHTop(
                status.getAssignment(k),
                heuristic == Heuristic::MCA ? 1 : 2,
                partialAssignments
            );
        }
        sortBigH(bigH, heuristic);
        if(debug){
            status.print();
        }
    }
}

std::pair<int, Assignment> SCMAPD::extractBigHTop() {
    // top() refers to tasks, [0] to PartialAssignment (and so waypoints) (pair<int, ptr>)
    // thanks to shared pointer, the heap does not destroy the object and partialAssignmentsPtr doesn't throw SIGSEG
    auto& [taskId, partialAssignments] = bigH.front();
    auto candidateAssignment{std::move(partialAssignments[0])};

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
    sortPA(partialAssignments, v);

    for (int i = 0 ; i < v ; ++i) {
        auto& targetPA = partialAssignments[i];
        if(targetPA.hasConflicts(status.getConstraints()[fixedAssignment.getIndex()])) {
            targetPA.internalUpdate(status.getConstraints(), status.getTasks(),
                                    status.getAmbientMapInstance(), false);
            // todo min search on first v elements or everyone?
            sortPA(partialAssignments, v);
            // restart
            i = 0;
        }
    }
}

void SCMAPD::sortBigH(BigH &bigH, Heuristic heuristic) {
    std::function<bool(const SmallH&,const SmallH&)> comparator;

    switch(heuristic){
        case Heuristic::MCA:
                comparator = [](const SmallH& a, const SmallH& b) -> bool {return a.second[0] < b.second[0];};
        break;
        case Heuristic::RMCA_A:
            comparator =
                [](const SmallH& a, const SmallH& b) -> bool {
                    auto aVal = a.second[0].getMCA() - a.second[1].getMCA();
                    auto bVal = b.second[0].getMCA() - b.second[1].getMCA();

                    return aVal > bVal;
                };
        break;
        case Heuristic::RMCA_R:
            comparator = [](const SmallH& a, const SmallH& b) -> bool {
                    auto aVal = a.second[0].getMCA() / a.second[1].getMCA();
                    auto bVal = b.second[0].getMCA() / b.second[1].getMCA();

                    return aVal > bVal;
                };
        break;
    }

    std::iter_swap(
            bigH.begin(),
            std::min_element(bigH.begin(), bigH.end(), comparator)
    );
}

void SCMAPD::printResult() const{
    auto buildPathString = [](const std::vector<Coord>& path){
        static constexpr std::string_view pattern = "({},{})->";

        std::string result{};
        result.reserve(pattern.size() * path.size());

        for(const auto& pos : path){
            result.append(fmt::format(pattern, pos.row, pos.col));
        }

        result.resize(result.size() - 2);
        return result;
    };

    fmt::print("agent\tcost\tpath\n");
    for(const auto& a: status.getAssignments()){
        fmt::print("{}\t{}\t{}\n", a.getIndex(), a.getPath().size(), buildPathString(a.getPath()));
    }
}

void SCMAPD::sortPA(std::vector<Assignment> &pa, int v) {
    for(int i = 0 ; i < v ; ++i){
        std::iter_swap(pa.begin()+i, std::min_element(pa.begin()+i, pa.end()));
    }
}

void SCMAPD::printCheckMessage() const{
    if(!status.printCollisions()){
        fmt::print(fmt::emphasis::bold | fg(fmt::color::green), "No collisions\n");
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
        assert(instance.agents()[i] == robots[i].getStartPosition());
    }
    assert(instance.tasks().size() == tasks.size());
    for (int i = 0 ; i < instance.tasks().size() ; ++i){
        const auto& t = tasks[i].getCoordinates();
        assert(instance.tasks()[i] == t);
    }
#endif

    return {std::move(instance), std::move(robots), std::move(tasks), heuristic, false};
}
