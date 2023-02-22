#include <boost/program_options.hpp>
#include <string>
#include <iostream>
#include "SCMAPD.hpp"
#include "utils.hpp"

int main(int argc, char* argv[]){
    namespace po = boost::program_options;
    using std::string;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")

        // params for the input instance && experiment settings
        ("m", po::value<string>()->required(), "input file for map")
        ("dm", po::value<string>()->required(), "distance matrix file")
        ("a", po::value<string>()->required(), "agents file")
        ("t", po::value<string>()->required(), "tasks file")
        ("h", po::value<string>()->required(), "heuristic")
        ("s", po::value<string>()->required(), "pathfinding strategy")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << '\n';
        return 1;
    }

    po::notify(vm);

    auto distanceMatrixFile{vm["dm"].as<string>()};
    auto gridFile{vm["m"].as<string>()};
    auto robotsFile{vm["a"].as<string>()};
    auto tasksFile{vm["t"].as<string>()};
    auto heur{utils::getHeuristic(vm["h"].as<string>())};
    auto strategy{utils::getStrategy(vm["s"].as<string>())};

    SCMAPD scmapd{
        loadData(
            robotsFile, tasksFile, gridFile, distanceMatrixFile, heur, strategy
        )
    };
    scmapd.solve(10, 5, Objective::MAKESPAN);
    scmapd.printResult();

    scmapd.printCheckMessage();

    return 0;
}