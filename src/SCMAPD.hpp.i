#include <filesystem>

template<Heuristic heuristic>
SCMAPD<heuristic>::SCMAPD(DistanceMatrix && distanceMatrix,
               std::vector<Robot> && robots) :
    distanceMatrix(std::move(distanceMatrix)),
    assignment(std::move(robots))
    {}

template<>
SequenceOfReadyTasks SCMAPD<Heuristic::MCA>::insert(const Task &task, const SequenceOfReadyTasks &taskSequence) {
    // todo complete this
    return {}
}

bool comparePartialAssignment::operator()(const Robot& a, const Robot& b) {
    return !(a.getTtd() <= b.getTtd()) ;
}
