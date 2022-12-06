#include <boost/program_options.hpp>
#include <string>
#include <iostream>
#include "SCMAPD.hpp"
#include "utils.hpp"
#include "ambient/AmbientMap.h"
#include "ambient/AmbientMapInstance.h"

int main(int argc, char* argv[]){
    cmapd::AmbientMap map("grid.txt");

    auto robots{utils::loadRobots("data/0.agents", map.columns_number())};
    auto tasks{utils::loadTasks("data/0.tasks", map.columns_number())};

    auto rawMatrix = cnpy::npy_load("data/distance_matrix.npy");
    auto nCols = rawMatrix.shape[1];

    cmapd::AmbientMapInstance instance(
        map,
        {robots.begin(), robots.end()},
        {tasks.begin(), tasks.end()},
        {std::move(rawMatrix), static_cast<unsigned int>(nCols)}
    );

    SCMAPD scmapd{std::move(instance), std::move(robots), std::move(tasks), Heuristic::RMCA_A};
    scmapd.solve(Heuristic::HEUR, 10);

    namespace po = boost::program_options;
    using std::string;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")

            // params for the input instance && experiment settings
            ("map,m", po::value<string>()->required(), "input file for map")
            ("agents,a", po::value<string>()->required(), "input file for agents")
//            ("output,o", po::value<string>(), "output file for statistics")
//            ("outputPaths", po::value<string>(), "output file for paths")
            ("agentNum,k", po::value<int>()->default_value(0), "number of agents")
//            ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
//            ("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
//            ("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")
//
//            ("sipp", po::value<bool>()->default_value(1), "using SIPP as the low-level solver")
            ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << '\n';
        return 1;
    }

    cmapd::AmbientMap ambientMap{vm["map"].as<std::string>()};

    po::notify(vm);

    return 0;
}