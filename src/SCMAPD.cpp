#include <algorithm>
#include <functional>
#include "SCMAPD.hpp"
#include "Assignment.hpp"
#include "fmt/color.h"
#include "BigH.hpp"

SCMAPD::SCMAPD(cmapd::AmbientMapInstance &&ambientMapInstance, std::vector<Assignment> &&robots,
               std::vector<Task> &&tasksVector, Heuristic heuristic, bool debug) :
    status(std::move(ambientMapInstance), std::move(robots), std::move(tasksVector)),
    bigH{status, heuristic},
    debug{debug}
    {
    if(debug)
        status.print();
    }

void SCMAPD::solve(TimeStep cutOffTime) {
    // extractBigHTop takes care of tasks indices removal
    while( !bigH.empty() ){
        assert(!status.checkCollisions());
        auto [taskId, candidateAssignment] = bigH.extractAndDestroy();
        auto k = status.update(std::move(candidateAssignment));

        bigH.updateOtherPAs(k, status, taskId);
        bigH.updateSmallHTop(k, status);
        if(debug){
            status.print();
        }
    }
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

void SCMAPD::printCheckMessage() const{
    if(!status.printCollisions()){
        fmt::print(fmt::emphasis::bold | fg(fmt::color::green), "No collisions\n");
    }
}

SCMAPD loadData(const std::filesystem::path &agentsFile, const std::filesystem::path &tasksFile,
                       const std::filesystem::path &gridFile, const std::filesystem::path &distanceMatrixFile,
                       Heuristic heuristic) {
    DistanceMatrix dm(cnpy::npy_load(distanceMatrixFile));

    auto robots{loadAssignments(agentsFile)};
    auto tasks{loadTasks(tasksFile, dm)};

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
