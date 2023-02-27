//
// Created by nicco on 21/02/2023.
//

#ifndef SIMULTANEOUS_CMAPD_NOSOLUTION_HPP
#define SIMULTANEOUS_CMAPD_NOSOLUTION_HPP

#include <exception>
#include <string_view>

class NoSolution : public std::exception{
public:
    inline NoSolution() : msg{"Not possible to find solution"} {};
    [[nodiscard]] inline const char * what() const noexcept override{
        return msg.data();
    }
private:
    std::string_view msg;
};

#endif //SIMULTANEOUS_CMAPD_NOSOLUTION_HPP
