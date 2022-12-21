#include <algorithm>
#include "SmallH.hpp"

SmallH::SmallH(const Status &status, const Task &task, int v) :
        paVec{initializePASet(status, task.index, v)},
        taskId{task.index},
        v{v}
    {
        sortVTop();
    }

std::vector<Assignment> SmallH::initializePASet(const Status &status, int taskId, int v) {
    const auto& tasks = status.getTasks();
    std::vector<Assignment> partialAssignments{};
    partialAssignments.reserve(tasks.size());

    for (const auto& a : status.getAssignments()){
        Assignment pa {a.getStartPosition(), a.getIndex(), a.getCapacity()};
        pa.addTask(status.getAmbientMapInstance(), status.getConstraints(), taskId, status.getTasks(), status.getAssignments());
        partialAssignments.push_back(std::move(pa));
    }

    return partialAssignments;
}

std::pair<int, Assignment> SmallH::extractTopAndDestroy() {
    assert(!paVec.empty());
    std::pair<int, Assignment> tmp {taskId, std::move(*paVec.begin())};
    paVec.clear();
    return tmp;
}

void SmallH::updateTopElements(const Assignment &a, const Status &status) {
    for (int i = 0 ; i < v ; ++i) {
        auto targetIt = paVec.begin() + i;

        // same agent
        if(a.getIndex() == targetIt->getIndex()){
            continue;
        }

        if(Assignment::conflictsWithPath(targetIt->getPath(), a.getPath())){
            targetIt->internalUpdate(status.getConstraints(), status.getTasks(), status.getAmbientMapInstance(), false,
                                     status.getAssignments());
            sortVTop();
            // restart
            i = 0;
        }
    }
}

TimeStep SmallH::getTopMCA() const{
    assert(!paVec.empty());
    return paVec.cbegin()->getMCA();
}

void SmallH::sortVTop() {
    for (int i = 0 ; i < std::min(v, static_cast<int>(paVec.size())) ; ++i) {
        auto it = paVec.begin() + i;
        auto minElIt = std::min_element(it, paVec.end());
        std::iter_swap(it, minElIt);
    }
}

Assignment &SmallH::find(int id) {
    auto result = std::find_if(paVec.begin(), paVec.end(), [&id](const Assignment& el){return el.getIndex() == id;});
    assert(result != paVec.end());
    return *result;
}

void SmallH::addTaskToAgent(int k, int otherTaskId, const Status &status) {
    auto& target = find(k);
    assert(target.getIndex() == k);
    target.addTask(status.getAmbientMapInstance(), status.getConstraints(), otherTaskId, status.getTasks(),
                   status.getAssignments());
    sortVTop();
}
