#include <boost/program_options.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "nlohmann/json.hpp"
#include "Simulation/RunningAgent.hpp"
#include "AmbientMap.hpp"

namespace po = boost::program_options;
namespace fs = std::filesystem;
using json = nlohmann::json;

int main(int argc, char* argv[]){

    // Declare the supported options.
    po::options_description desc("Allowed options");

    desc.add_options()
        ("help", "produce help message")

        // params for the input instance && experiment settings
        ("m", po::value<std::string>()->required(), "input file for map")
        ("dm", po::value<std::string>()->required(), "distance matrix file")
        ("j", po::value<std::string>()->required(), "input json");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << '\n';
        return 1;
    }

    po::notify(vm);

    fs::path jPath{vm["j"].as<std::string>()};
    std::ifstream jFile(jPath);
    json j;
    jFile >> j;

    AmbientMap ambient{vm["m"].as<std::string>(), vm["dm"].as<std::string>()};
    const auto runningAgents = loadPlansFromJson(j, ambient.getDistanceMatrix());

    return 0;
}