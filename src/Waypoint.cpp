#include "Waypoint.hpp"
#include "Task.hpp"

Waypoint::operator std::string() const {
    auto taskString = taskIndex.has_value() ? fmt::format("taskId: {}", taskIndex.value()) : "no task";
    return fmt::format("[pos: {}, demand: {}, {}]",
        position, static_cast<int>(demand), taskString);
}

Waypoint::Waypoint(CompressedCoord position, Demand demand, int taskIndex) :
    position(position),
    demand(demand),
    taskIndex(taskIndex) {
    assert(demand != Demand::END);
}

TimeStep
Waypoint::update(TimeStep newArrivalTime) {
    realArrivalTime = newArrivalTime;
    return realArrivalTime.value();
}

TimeStep Waypoint::getRealArrivalTime() const {
    return realArrivalTime.value();
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
    return taskIndex.value();
}

TimeStep Waypoint::getDelay(const std::vector<Task>& tasks) const {
    return realArrivalTime.value() - tasks[taskIndex.value()].idealGoalTime;
}

void Waypoint::reset() {
    realArrivalTime.reset();
}

Waypoint getTaskPickupWaypoint(const Task& task){
    return {task.startLoc, Demand::PICKUP, task.index};
}

Waypoint getTaskDeliveryWaypoint(const Task& task){
    return {task.goalLoc, Demand::DELIVERY, task.index};
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
            {"arrival_time", wp.getRealArrivalTime()},
            {"task_id", wp.getTaskIndex()}
        });
    }

    return j;
}

void WaypointsList::reset() {
    std::for_each(begin(), end(), [](Waypoint& wp){wp.reset();});
}
