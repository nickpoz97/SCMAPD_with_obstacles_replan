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
        ("m", po::value<string>()->default_value("./data/grid.txt"), "input file for map")
        ("dm", po::value<string>()->default_value("./data/distance_matrix.npy"), "distance matrix file")
        ("a", po::value<string>()->required(), "agents file")
        ("t", po::value<string>()->required(), "tasks file")
        ("h", po::value<string>()->required(), "heuristic")
        ("obj", po::value<string>()->required(), "optimization objective")
        ("metric", po::value<string>()->required(), "optimization metric")
        ("cutoff", po::value<int>()->default_value(10), "optimization threshold in seconds")
        ("nt", po::value<int>()->required(), "number of tasks to optimize at each iteration")
        ("mtd", po::value<string>()->required(), "optimization method")
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
    auto objective{utils::getObjective(vm["obj"].as<string>())};
    auto cutoffTime{vm["cutoff"].as<int>()};
    auto nt{vm["nt"].as<int>()};
    auto mtd{utils::getMethod(vm["mtd"].as<string>())};
    auto metric{utils::getMetric(vm["metric"].as<string>())};

    SCMAPD scmapd{
        loadData(
            robotsFile, tasksFile, gridFile, distanceMatrixFile, heur, PathfindingStrategy::UNBOUNDED
        )
    };
    scmapd.solve(cutoffTime, nt, objective, mtd, metric);
    scmapd.printResult();

    //scmapd.printCheckMessage();

    return 0;
}