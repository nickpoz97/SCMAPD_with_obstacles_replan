//
// Created by nicco on 02/02/2023.
//

#ifndef SIMULTANEOUS_CMAPD_NOPATHEXCEPTION_HPP
#define SIMULTANEOUS_CMAPD_NOPATHEXCEPTION_HPP

#include <exception>
#include <string_view>

class NoPathException : public std::exception{
public:
    inline NoPathException() : msg{"Impossible to find path"} {};
    [[nodiscard]] inline const char * what() const noexcept override{
        return msg.data();
    }
private:
    std::string_view msg;
};

#endif //SIMULTANEOUS_CMAPD_NOPATHEXCEPTION_HPP
