//
// Created by nicco on 20/12/2022.
//

#ifndef SIMULTANEOUS_CMAPD_HEAP_HPP
#define SIMULTANEOUS_CMAPD_HEAP_HPP

#include <list>
#include <functional>
#include <stdexcept>

template <typename T>
class Heap{
public:
    using CompareType = std::function<bool(const T&,const T&)>;
    explicit Heap(const CompareType& cmp);

    T& find(int id);
    void restoreRoot();
    void sortVTop(int v);
    T extractTop();
private:
    std::list<T> list;
    CompareType compare;
};

#endif //SIMULTANEOUS_CMAPD_HEAP_HPP
