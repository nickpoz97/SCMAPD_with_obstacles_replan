#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include <utility>
#include <boost/tokenizer.hpp>
#include"Instance.h"

bool Instance::validMove(int curr, int next) const
{
	if (next < 0 || next >= map_size)
		return false;
	if (my_map[next])
		return false;
	return getManhattanDistance(curr, next) < 2;
}


list<int> Instance::getNeighbors(int curr, int t) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
	for (int next : candidates) {
        // check if move is valid and no spawned obstacles collision
		if (validMove(curr, next) && !spawnedObstacles.contains({t+1, next}))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

list<int> Instance::getNeighbors(int curr) const
{
    list<int> neighbors;
    int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
    for (int next : candidates) {
        if (validMove(curr, next))
            neighbors.emplace_back(next);
    }
    return neighbors;
}

Instance::Instance(vector<bool> map, vector<vector<int>> agents, int nRows, int nCols,
                   SpawnedObstaclesSet spawnedObstacles) :
        num_of_rows{nRows},
        num_of_cols{nCols},
        map_size(nRows * nCols),
        my_map{std::move(map)},
        num_of_agents{static_cast<int>(agents.size())},
        locations{std::move(agents)},
        spawnedObstacles{std::move(spawnedObstacles)}
    {}

