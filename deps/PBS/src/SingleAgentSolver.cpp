#include "SingleAgentSolver.h"


void SingleAgentSolver::compute_heuristics()
{
	struct Node
	{
		int location;
		int value;

		Node() = default;
		Node(int location, int value) : location(location), value(value) {}
		// the following is used to comapre nodes in the OPEN list
		struct compare_node
		{
			// returns true if n1 > n2 (note -- this gives us *min*-heap).
			bool operator()(const Node& n1, const Node& n2) const
			{
				return n1.value >= n2.value;
			}
		};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, && then highest g-val)
	};

	vector <int> mh;
	for (int i = 0; i < locs.size(); i++) {
		mh.resize(instance.map_size, MAX_TIMESTEP);
	
		// generate a heap that can save nodes (&& a open_handle)
		boost::heap::pairing_heap< Node, boost::heap::compare<Node::compare_node> > heap;

		Node root(locs[i], 0);
		mh[locs[i]] = 0;
		heap.push(root);  // add root to heap
		while (!heap.empty())
		{
			Node curr = heap.top();
			heap.pop();
			for (int next_location : instance.getNeighbors(curr.location))
			{
				if (mh[next_location] > curr.value + 1)
				{
					mh[next_location] = curr.value + 1;
					Node next(next_location, curr.value + 1);
					heap.push(next);
				}
			}
		}
		my_heuristic.insert({locs[i], mh});
	}
}