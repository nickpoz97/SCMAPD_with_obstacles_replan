#include "Heap.hpp"

template<typename T>
Heap<T>::Heap(const Heap::CompareType &cmp) : list{}, compare{cmp} {}

template<typename T>
T &Heap<T>::find(int id) {
    auto result = std::find_if(list.begin(), list.end(), [&id](const T& el){ el.getIndex() == id;});
    assert(result == list.end());
    return *result;
}

template<typename T>
void Heap<T>::sortVTop(int v) {
    auto it = list.begin();
    for (int i = 0 ; i < v ; ++v){
        auto result = std::min_element(it, list.end(), compare);
        std::swap(it, result);
    }
}

template<typename T>
void Heap<T>::restoreRoot() {
    sortVTop(1);
}

template<typename T>
T Heap<T>::extractTop() {
    assert(!list.empty());
    auto tmp = std::move(list.front());
    list.pop_front();
    restoreRoot();
    return tmp;
}
