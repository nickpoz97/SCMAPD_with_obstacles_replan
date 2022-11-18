#ifndef SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
#define SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP

#include <unordered_map>
#include "TypeDefs.hpp"
#include "Robot.hpp"

enum class Heuristic{
    MCA,
    RMCA_A,
    RMCA_R
};

template <Heuristic heuristic>
class PartialAssignment {
public:
    PartialAssignment(const Robot& robot, const Task& task, const DistanceMatrix& distanceMatrix);

    [[nodiscard]] unsigned getTaskIndex() const;
    [[nodiscard]] TimeStep getTtd() const;
    Waypoints releaseWaypoints();
private:
    Robot robot;
    const unsigned taskIndex;

    static Robot insert(const Robot& robot, const Task& task, const DistanceMatrix& distanceMatrix);

    static Robot initialize(const Robot &robot, const Task &task, const DistanceMatrix &distanceMatrix);

    static Robot update(const Robot &robot, const Task &task, const DistanceMatrix &vector);
};


#endif //SIMULTANEOUS_CMAPD_PARTIALASSIGNMENT_HPP
