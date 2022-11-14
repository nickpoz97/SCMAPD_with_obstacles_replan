#include "SCMAPD.hpp"
#include "utils.hpp"

int main(){
    SCMAPD<Heuristic::MCA> scmapd{
        utils::loadDistanceMatrix("data/distance_matrix.npy"),
        {Robot{3}, Robot{4}, Robot{7}}
    };

    return 0;
}