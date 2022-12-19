#include "SmallH.hpp"

SmallH::SmallH(const std::vector<Assignment> &agents, Task &&task, const cmapd::AmbientMapInstance &instance) :
        paSet{initializePASet(agents, task, instance)},
        task{task}
    {}

std::set<Assignment> SmallH::initializePASet(const std::vector<Assignment> &agents, const Task &task,
                                             const cmapd::AmbientMapInstance &instance) {
    std::set<Assignment> partialAssignments{};

    for (const auto& a : agents){
        Assignment pa {a.getStartPosition(), a.getIndex(), a.getCapacity()};
        pa.addTask(instance, {}, task);
        partialAssignments.insert(pa);
    }

    return partialAssignments;
}

std::pair<int, Assignment> SmallH::extractTop() {
    return {task.index, std::move(paSet.extract(paSet.begin()).value())};
}

const Task &SmallH::getTask() const{
    return task;
}

void SmallH::updateSmallHTop(const Assignment &a, int v, const Status &status) {
    for (int i = 0 ; i < v ; ++i) {
        for (auto& targetPA : paSet){
            if(targetPA.getIndex() == a.getIndex() && targetPA.hasConflicts(status.getConstraints()[a.getIndex()])){
                targetPA.internalUpdate(status.getConstraints(), status.getTasks(), status.getAmbientMapInstance(), false);
                // restart
                i = 0;
            }
        }
    }
}
