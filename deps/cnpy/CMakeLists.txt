cmake_minimum_required(VERSION 3.22)
project(CNPY)

set(CMAKE_CXX_STANDARD 17)

find_package(ZLIB REQUIRED)
file(GLOB CNPY_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_library(cnpy STATIC ${CNPY_SRC})

target_link_libraries(cnpy PRIVATE ZLIB::ZLIB)
target_include_directories(cnpy PUBLIC include ${ZLIB_INCLUDE_DIR})
