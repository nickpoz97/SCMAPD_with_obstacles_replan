project("Simultaneous CMAPD")

cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)

set(EXE cmapd)

file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
set(INCLUDE_DIRS include)

add_subdirectory(deps)
set(CNPY_INCLUDE_DIR "deps/cnpy/include")

add_executable(${EXE} ${SOURCES})
target_include_directories(${EXE} PUBLIC ${INCLUDE_DIRS} ${CNPY_INCLUDE_DIR})
target_link_libraries(${EXE} PRIVATE cnpy fmt::fmt)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(${EXE} PRIVATE ${Boost_LIBRARIES})
