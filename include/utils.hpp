#ifndef SIMULTANEOUS_CMAPD_UTILS_HPP
#define SIMULTANEOUS_CMAPD_UTILS_HPP

#include <NewTypes.hpp>
#include <filesystem>

#include <fmt/core.h>
#include <fmt/printf.h>

namespace utils{
    inline std::pair<std::string, std::string> buildDivider(std::string_view name){
        static constexpr std::string_view divider {"################{}################"};
        auto firstDiv = fmt::format(" {} ", name);
        auto lastDiv = std::string(firstDiv.size(), '#');

        return {
            fmt::format(divider, firstDiv),
            fmt::format(divider, lastDiv)
        };
    }

    inline std::string objContainerString(const auto& objContainer){
        std::string result{};
        for(const auto& obj : objContainer){
            result += fmt::format("{},", static_cast<std::string>(obj));
        }
        // remove last comma
        if(!result.empty()) {
            result.pop_back();
            return fmt::format("{{{}}}", result);
        }
        return result;
    }

    inline Heuristic getHeuristic(const std::string& hString)
    {
        if (hString == "MCA") return Heuristic::MCA;
        if (hString == "RMCA_A") return Heuristic::RMCA_A;
        if (hString == "RMCA_R") return Heuristic::RMCA_R;
        throw std::runtime_error("Invalid Heuristic option");
    }

    inline PathfindingStrategy getStrategy(const std::string& sString)
    {
        if (sString == "LAZY") return PathfindingStrategy::LAZY;
        if (sString == "EAGER") return PathfindingStrategy::EAGER;
        if (sString == "FORWARD_ONLY") return PathfindingStrategy::FORWARD_ONLY;
        if (sString == "UNBOUNDED") return PathfindingStrategy::UNBOUNDED;
        throw std::runtime_error("Invalid Strategy option");
    }

    inline Objective getObjective(const std::string& objString)
    {
        if(objString == "MAKESPAN") return Objective::MAKESPAN;
        if(objString == "TTD") return Objective::TTD;
        throw std::runtime_error("Invalid Objective option");
    }
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
