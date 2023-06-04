#pragma once

#include "common.h"
#include "SpawnedObstacle.hpp"

// Currently only works for undirected unweighted 4-nighbor grids
class Instance 
{
public:
	int num_of_cols;
	int num_of_rows;
	int map_size;

	// enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size
    Instance(
        vector<bool> map,
        vector<vector<int>> agents,
        int nRows,
        int nCols,
        SpawnedObstaclesSet spawnedObstacles
    );


    inline bool isObstacle(int loc) const { return my_map[loc]; }
		inline bool validMove(int curr, int next) const;
		list<int> getNeighbors(int curr, int t) const;
		list<int> getNeighbors(int curr) const;


		inline int linearizeCoordinate(int row, int col) const { return ( this->num_of_cols * row + col); }
		inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
		inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
		inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id % this->num_of_cols); }
		inline int getCols() const { return num_of_cols; }

		inline int getManhattanDistance(int loc1, int loc2) const
		{
			int loc1_x = getRowCoordinate(loc1);
			int loc1_y = getColCoordinate(loc1);
			int loc2_x = getRowCoordinate(loc2);
			int loc2_y = getColCoordinate(loc2);
			return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
		}

		inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2) const
		{
			return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
		}

	int getDegree(int loc) const
	{
		assert(loc >= 0 && loc < map_size && !my_map[loc]);
		int degree = 0;
		if (0 < loc - num_of_cols && !my_map[loc - num_of_cols])
			degree++;
		if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
			degree++;
		if (loc % num_of_cols > 0 && !my_map[loc - 1])
			degree++;
		if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
			degree++;
		return degree;
	}

	int getDefaultNumberOfAgents() const { return num_of_agents; }

private:
	  // int moves_offset[MOVE_COUNT];
	  vector<bool> my_map;
	  string map_fname;
	  string agent_fname;

	  int num_of_agents;
	  vector<vector<int>> locations;

      SpawnedObstaclesSet spawnedObstacles{};

    // add this obsatcle only if the map is still connected
    // run BFS to find a path between start and goal, return true if a path exists.

    // Class  SingleAgentSolver can access private members of Node
	  friend class SingleAgentSolver;
};

