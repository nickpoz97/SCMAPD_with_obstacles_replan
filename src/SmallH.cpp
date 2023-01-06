#include <algorithm>
#include "SmallH.hpp"

SmallH::SmallH(const std::vector<AgentInfo> &agentsInfos, int taskId, int v, const Status &status) :
        heap{initializePASet(agentsInfos, taskId, status)},
        taskId{taskId},
        v{v}
    {
        sortVTop();
    }

SmallHFibHeap
SmallH::initializePASet(const std::vector<AgentInfo> &agentsInfos, int taskId, const Status &status) {
    SmallHFibHeap partialAssignments{};

    for (const auto& aInfo : agentsInfos){
        partialAssignments.emplace(aInfo.startPos, aInfo.index, aInfo.capacity, taskId, status);
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

        if(Assignment::conflictsWithPath(targetIt->extractPath(), a.extractPath())){
            targetIt->internalUpdate(status.getTasks(), <#initializer#>);
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
    target.addTask(otherTaskId,
                   status.getAssignments());
    sortVTop();
}
