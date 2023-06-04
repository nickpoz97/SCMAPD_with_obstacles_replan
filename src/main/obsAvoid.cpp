#include <boost/program_options.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "nlohmann/json.hpp"
#include "Simulation/RunningAgent.hpp"
#include "AmbientMap.hpp"
#include "Simulation/Simulator.hpp"

namespace po = boost::program_options;
namespace fs = std::filesystem;
using json = nlohmann::json;

std::istream& operator>> (std::istream& in, Strategy& strategy)
{
    std::string token;
    in >> token;
    if (token == "RE_PLAN")
        strategy = Strategy::RE_PLAN;
    else if (token == "WAIT")
        strategy = Strategy::WAIT;
    else
        in.setstate(std::ios_base::failbit);
    return in;
}

int main(int argc, char* argv[]){

    // Declare the supported options.
    po::options_description desc("Allowed options");

    Strategy strategy;
    desc.add_options()
        ("help", "produce help message")

        // params for the input instance && experiment settings
        ("m", po::value<std::string>()->required(), "input file for map")
        ("dm", po::value<std::string>()->required(), "distance matrix file")
        ("plans", po::value<std::string>()->required(), "plans json file")
        ("obstacles", po::value<std::string>()->required(), "obstacles json file")
        ("strategy", po::value<Strategy>(&strategy)->required(), "obstacles json file")
        ("out", po::value<std::string>()->required(), "results json file");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << '\n';
        return 1;
    }

    po::notify(vm);

    fs::path plansPath{vm["plans"].as<std::string>()};
    std::ifstream plansJsonFile(plansPath);
    json plansJson;
    plansJsonFile >> plansJson;

    fs::path obsPath{vm["obstacles"].as<std::string>()};
    std::ifstream obsJsonFile(obsPath);
    json obsJson;
    obsJsonFile >> obsJson;

    AmbientMap ambient{vm["m"].as<std::string>(), vm["dm"].as<std::string>()};
    const auto runningAgents = loadPlansFromJson(plansJson, ambient.getDistanceMatrix());

    Simulator simulator{
        loadPlansFromJson(plansJson, ambient.getDistanceMatrix()),
        {computeSeed(runningAgents), obsJson},
        ambient
    };

    simulator.simulate(strategy);
    simulator.printResults(vm["out"].as<std::string>(), plansJson);

    return 0;
}