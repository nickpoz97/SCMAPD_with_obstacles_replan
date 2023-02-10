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
        ("eager", po::bool_switch()->default_value(false), "use eager pathfinding strategy instead of lazy")
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
    auto heurString{vm["h"].as<string>()};
    auto strategy = vm["eager"].as<bool>() ? Strategy::EAGER : Strategy::LAZY;

#ifndef NDEBUG
    fmt::print("Using {} strategy\n", strategy == Strategy::EAGER ? "eager" : "lazy");
#endif

    SCMAPD scmapd{
        loadData(
            robotsFile, tasksFile, gridFile, distanceMatrixFile, utils::getHeuristicFromString(heurString), strategy
        )
    };
    scmapd.solve(10);
    scmapd.printResult();

    scmapd.printCheckMessage();

    return 0;
}