//
// Created by nicco on 31/05/2023.
//

#ifndef SIMULTANEOUS_CMAPD_SPAWNEDOBSTACLE_HPP
#define SIMULTANEOUS_CMAPD_SPAWNEDOBSTACLE_HPP

#include <unordered_set>
#include <boost/functional/hash.hpp>

struct SpawnedObstacle{
    int t;
    int position;

    bool operator==(const SpawnedObstacle& other) const = default;
};

template<>
struct std::hash<SpawnedObstacle>{
    inline size_t operator()(const SpawnedObstacle& so) const{
        std::size_t seed = 0;
        boost::hash_combine(seed, so.t);
        boost::hash_combine(seed, so.position);
        return seed;
    }
};

using SpawnedObstaclesSet = std::unordered_set<SpawnedObstacle>;

#endif //SIMULTANEOUS_CMAPD_SPAWNEDOBSTACLE_HPP
