//
// Created by nicco on 21/02/2023.
//

#ifndef SIMULTANEOUS_CMAPD_NOTPOSSIBLEOPTIMIZATION_HPP
#define SIMULTANEOUS_CMAPD_NOTPOSSIBLEOPTIMIZATION_HPP

#include <exception>
#include <string_view>

class NotPossibleOptimization : public std::exception{
public:
    inline NotPossibleOptimization() : msg{"Not possible to optimize"} {};
    [[nodiscard]] inline const char * what() const noexcept override{
        return msg.data();
    }
private:
    std::string_view msg;
};

#endif //SIMULTANEOUS_CMAPD_NOTPOSSIBLEOPTIMIZATION_HPP
