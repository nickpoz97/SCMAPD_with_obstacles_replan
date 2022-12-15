//
// Created by nicco on 15/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_ITERABLECONSTRAINTS_HPP
#define SIMULTANEOUS_CMAPD_ITERABLECONSTRAINTS_HPP

#include <vector>
#include "Assignment.hpp"

class IterableConstraints {
public:
    IterableConstraints(const std::vector<Assignment>& assignments, int k);

    struct iterator {
        using iterator_category = std::input_iterator_tag;
        using difference_type = int;
        using value_type = cmapd::Constraint;
        using pointer = std::vector<cmapd::Constraint>::const_iterator;
        using reference = const value_type &;

        iterator(pointer it, const std::vector<Assignment> &assignments, int k);

        reference operator*() const { return *it; }

        pointer operator->() { return it; }

        iterator &operator++() {
            increment();
            return *this;
        }

        iterator operator++(int) {
            iterator tmp = *this;
            increment();
            return tmp;
        }

        friend bool operator==(const iterator &a, const iterator &b) { return a.it == b.it; }
        friend bool operator!=(const iterator &a, const iterator &b) { return a.it != b.it; }

    private:
        pointer it;
        int k;
        const std::vector<Assignment>& assignments;
        int actualAgent;

        void increment();
        void adjust();
    };

    iterator begin() {
        return {assignments.begin()->getConstraints().begin(), assignments, k};
    }
    iterator end(){
        return {assignments.rbegin()->getConstraints().end(), assignments, k};
    }
private:
    const std::vector<Assignment>& assignments;
    int k;
};


#endif //SIMULTANEOUS_CMAPD_ITERABLECONSTRAINTS_HPP
