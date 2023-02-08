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

    inline Heuristic getHeuristicFromString(std::string_view heurString){
        if(heurString == "MCA"){
            return Heuristic::MCA;
        }
        if(heurString == "RMCA_R"){
            return Heuristic::RMCA_R;
        }
        if(heurString == "RMCA_A"){
            return Heuristic::RMCA_A;
        }
        throw std::runtime_error("Wrong heuristic choice");
    }
}

#endif //SIMULTANEOUS_CMAPD_UTILS_HPP
