//
// Created by nicco on 02/06/2023.
//

#ifndef SIMULTANEOUS_CMAPD_FIXEDAGENTSTEP_HPP
#define SIMULTANEOUS_CMAPD_FIXEDAGENTSTEP_HPP

#include <functional>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <boost/functional/hash.hpp>

class FixedPath : public std::vector<int>{
public:
    [[nodiscard]] inline int getPos(int t) const{
        t = std::min(static_cast<int>(this->size()), t);

        return this->operator[](t);
    }
};

class FixedPaths: public std::vector<FixedPath>{
public:
    [[nodiscard]] inline bool conflict(int fromPos, int toPos, int fromT, bool isFinal) const{
        if(this->empty()){
            return false;
        }

        return std::ranges::any_of(
            *this,
            [=](const FixedPath& path){
                if(path.empty()){
                    return false;
                }

                auto toT = fromT + 1;

                // go to occupied location
                bool nodeConflict = path.getPos(toT) == toPos;

                // switch position
                bool edgeConflict = fromT < (std::ssize(path) - 1) && path.getPos(fromT) == toPos && path.getPos(toT) == fromPos;

                // staying on other path
                bool dockConflict = isFinal && toT < path.size() &&
                    std::ranges::any_of(
                        path.cbegin() + toT,
                        path.cend(),
                        [toPos](int otherPos){return toPos == otherPos;}
                    );

                return nodeConflict || edgeConflict || dockConflict;
            }
        );
    }
};

#endif //SIMULTANEOUS_CMAPD_FIXEDAGENTSTEP_HPP
