#include <Task.hpp>

std::size_t TaskHasher::operator()(const Task& task) const{
    // https://stackoverflow.com/questions/38965931/hash-function-for-3-integers

    auto a = task.startLoc;
    auto b = task.goalLoc;
    auto c = task.releaseTime;

    auto Hab = ((a + b) * (a + b + 1) + b) / 2;

    return ((Hab + c) * (Hab + c + 1) + c) / 2;
}

bool operator==(const Task &t1, const Task &t2) {
        return t1.startLoc == t2.startLoc &&
               t1.goalLoc == t2.goalLoc &&
               t1.releaseTime == t2.releaseTime;
}
