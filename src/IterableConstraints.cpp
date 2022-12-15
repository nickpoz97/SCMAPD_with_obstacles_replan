#include "IterableConstraints.hpp"

IterableConstraints::IterableConstraints(const std::vector<Assignment> &assignments, int k) :
        assignments{assignments},
        k{k}
    {}

IterableConstraints::iterator::iterator(
    IterableConstraints::iterator::pointer it,
    const std::vector<Assignment> &assignments, int k
    )  :
    assignments{assignments},
    it{it},
    k{k},
    actualAgent{it->agent}
{
    adjust();
}

void IterableConstraints::iterator::increment() {
    ++it;
    adjust();
}

void IterableConstraints::iterator::adjust() {
    const auto& getAgentConstraints = [this](){return assignments[actualAgent].getConstraints();};

    if(it == getAgentConstraints().end() && actualAgent != assignments.rbegin()->getIndex()){
        ++actualAgent;
        it = getAgentConstraints().begin();
    }
}


