#include "Waypoint.hpp"
#include "Task.hpp"

Waypoint::operator CompressedCoord() const {return position;}

Waypoint::operator std::string() const {
    auto taskString = taskIndex.has_value() ? fmt::format("taskId: {}", taskIndex.value()) : "no task";
    return fmt::format("[pos: {}, demand: {}, {}]",
        position, static_cast<int>(demand), taskString);
}

Waypoint::Waypoint(const Task& task, Demand demand) :
    position(demand == Demand::DELIVERY ? task.goalLoc : task.startLoc),
    demand(demand),
    taskIndex(task.index),
    idealGoalTime(demand == Demand::DELIVERY ? std::optional<TimeStep>{task.idealGoalTime} : std::nullopt)
    {}

TimeStep
Waypoint::update(TimeStep newArrivalTime, TimeStep previousCumulatedDelay) {
    arrivalTime = newArrivalTime;
    auto localDelay = 0;
    if (demand == Demand::DELIVERY) {
        localDelay = newArrivalTime - *idealGoalTime;
    }
#ifndef NDEBUG
    if(localDelay < 0) { throw std::runtime_error("negative delay"); }
#endif
    cumulatedDelay = previousCumulatedDelay + localDelay;

    return cumulatedDelay;
}

TimeStep Waypoint::getCumulatedDelay() const {
    return cumulatedDelay;
}

TimeStep Waypoint::getArrivalTime() const {
    assert(arrivalTime.has_value());
    return *arrivalTime;
}

Waypoint::Waypoint(CompressedCoord robotStartPosition) :
    position{robotStartPosition},
    demand{Demand::END},
    taskIndex{std::nullopt}
    {}

CompressedCoord Waypoint::getPosition() const {
    return position;
}

Demand Waypoint::getDemand() const {
    return demand;
}

int Waypoint::getTaskIndex() const {
    assert(taskIndex.has_value());
    return *taskIndex;
}

TimeStep Waypoint::getDelay() const {
    assert(arrivalTime.has_value() && idealGoalTime.has_value());
    return *arrivalTime- *idealGoalTime;
}

Waypoint getTaskPickupWaypoint(const Task& task){
    return {task, Demand::PICKUP};
}

Waypoint getTaskDeliveryWaypoint(const Task& task){
    return {task, Demand::DELIVERY};
}

VerbosePath getWpCoords(const WaypointsList &wpList, const DistanceMatrix &dm) {
    VerbosePath wpCoords{};
    wpCoords.reserve(wpList.size());

    std::ranges::transform(
        wpList,
        std::back_inserter(wpCoords),
        [&dm](const auto& wp){return dm.from1Dto2D(wp.getPosition());}
    );
    return wpCoords;
}

nlohmann::json getWpsJson(const WaypointsList &wpList, const DistanceMatrix &dm){
    using namespace nlohmann;

    json j{};

    for(const auto& wp : wpList){
        if(wp.getDemand() == Demand::END){
            continue;
        }
        j.push_back({
            {"coords", static_cast<json>(dm.from1Dto2D(wp.getPosition())).dump()},
            {"demand", wp.getDemand()},
            {"arrival_time", wp.getArrivalTime()},
            {"task_id", wp.getTaskIndex()}
        });
    }

    return j;
}
