//
// Created by nicco on 12/11/2022.
//

#ifndef SIMULTANEOUS_CMAPD_SCMAPD_HPP
#define SIMULTANEOUS_CMAPD_SCMAPD_HPP

#include <filesystem>
#include <unordered_set>
#include "Robot.hpp"

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

struct comparePartialAssignment{
    bool operator()(const Robot& a, const Robot& b);
};

template<Heuristic heuristic>
class SCMAPD {
public:
    SCMAPD(
       DistanceMatrix && distanceMatrix,
       std::vector<Robot> && robots
    );
private:
    const DistanceMatrix distanceMatrix;
    std::vector<Robot> assignment;
    //std::unordered_set<Task> unassignedTasks;

    SequenceOfReadyTasks insert(const Task &task, const SequenceOfReadyTasks& taskSequence);
};

#include "../src/SCMAPD.hpp.i"

#endif //SIMULTANEOUS_CMAPD_SCMAPD_HPP
