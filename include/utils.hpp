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

    inline Heuristic getHeuristic(const std::string& hString){
        if (hString == "MCA") return Heuristic::MCA;
        if (hString == "RMCA_A") return Heuristic::RMCA_A;
        if (hString == "RMCA_R") return Heuristic::RMCA_R;
        throw std::runtime_error("Invalid Heuristic option");
    }

    inline Objective getObjective(const std::string& objString){
        if(objString == "MAKESPAN") return Objective::MAKESPAN;
        if(objString == "TTD") return Objective::TTD;
        if(objString == "TTT") return Objective::TTT;
        throw std::runtime_error("Invalid Objective option");
    }

    inline Method getMethod(const std::string& mtdString){
        if(mtdString == "WORST_TASKS") return Method::WORST_TASKS;
        if(mtdString == "WORST_AGENTS") return Method::WORST_AGENTS;
        if(mtdString == "RANDOM_TASKS") return Method::RANDOM_TASKS;
        throw std::runtime_error("Invalid Method option");
    }

    inline Metric getMetric(const std::string& mtdString){
        if(mtdString == "DELAY") return Metric::DELAY;
        if(mtdString == "ARRIVAL_TIME") return Metric::ARRIVAL_TIME;
        throw std::runtime_error("Invalid Metric option");
    }

    // frequencyValue, isNumerator
    inline std::pair<int, bool> getFrequency(const float freq){
        return freq == 0 ? std::pair{0, true} : (
            freq >= 1.0 ? std::pair{static_cast<int>(freq), true} : std::pair{static_cast<int>(1 / freq), false}
        );
    }
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
