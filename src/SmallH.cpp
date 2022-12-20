#include "SmallH.hpp"

SmallH::SmallH(const Status &status, const Task &task) :
        paSet{initializePASet(status, task.index)},
        taskId{task.index}
    {}

std::set<Assignment> SmallH::initializePASet(const Status &status, int taskId) {
    std::set<Assignment> partialAssignments{};

    for (const auto& a : status.getAssignments()){
        Assignment pa {a.getStartPosition(), a.getIndex(), a.getCapacity()};
        pa.addTask(status.getAmbientMapInstance(), {}, taskId, status.getTasks());
        partialAssignments.insert(std::move(pa));
    }

    return partialAssignments;
}

std::pair<int, Assignment> SmallH::extractTop() {
    return {taskId, std::move(paSet.extract(paSet.begin()).value())};
}

void SmallH::updateSmallHTop(const Assignment &a, int v, const Status &status) {
    for (int i = 0 ; i < v ; ++i) {
        auto targetIt = std::next(paSet.begin(), v);

        // same agent
        if(a.getIndex() == targetIt->getIndex()){
            continue;
        }

        if(targetIt->hasConflicts(status.getConstraints()[a.getIndex()])){
            auto targetPA = std::move(paSet.extract(targetIt).value());
            targetPA.internalUpdate(status.getConstraints(), status.getTasks(), status.getAmbientMapInstance(), false);
            paSet.insert(targetPA);
            // restart
            i = 0;
        }
    }
}

TimeStep SmallH::getTopMCA() const{
    return paSet.cbegin()->getMCA();
}
