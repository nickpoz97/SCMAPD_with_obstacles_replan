#include "common.h"

std::ostream& operator<<(std::ostream& os, const PBSPath& path)
{
	for (const auto& state : path)
	{
		os << state.location; // << "(" << state.is_single() << "),";
	}
	return os;
}


bool isSamePath(const PBSPath& p1, const PBSPath& p2)
{
	if (p1.size() != p2.size())
		return false;
	for (unsigned i = 0; i < p1.size(); i++)
	{
		if (p1[i].location != p2[i].location)
			return false;
	}
	return true;
}
