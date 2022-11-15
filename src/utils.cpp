#include <utils.hpp>
#include <cnpy.h>

DistanceMatrix utils::loadDistanceMatrix(const std::filesystem::path &distanceMatrixPath) {
    const cnpy::NpyArray distanceMatrixObj = cnpy::npy_load(distanceMatrixPath.generic_string());

    unsigned startCoordsSize = distanceMatrixObj.shape[0] * distanceMatrixObj.shape[1];
    unsigned endCoordsSize = distanceMatrixObj.shape[2] * distanceMatrixObj.shape[3];

    // distance matrix is considered double
    const auto* data = distanceMatrixObj.data<double>();
    DistanceMatrix distanceMatrix(startCoordsSize, std::vector<CompressedCoord>(endCoordsSize));

    for (unsigned i = 0 ; i < startCoordsSize ; ++i){
        for (unsigned j = 0 ; j < endCoordsSize ; ++j){
            distanceMatrix[i][j] = static_cast<int>(data[i*startCoordsSize + j]);
        }
    }

    return distanceMatrix;
}

PBS utils::buildPBS(
        const std::filesystem::path &mapPath,
        const std::filesystem::path &agentsPath,
        int nAgents,
        bool sipp,
        int screen
){
    srand((int)time(0));

    ///////////////////////////////////////////////////////////////////////////
    // load the instance
    Instance instance(mapPath.generic_string(), agentsPath.generic_string(), nAgents);

    srand(0);
    return PBS(instance, sipp, screen);
}

