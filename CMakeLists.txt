project("Simultaneous CMAPD")

cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)

set(EXE cmapd)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

add_subdirectory(deps/cnpy)
set(CNPY_INC "deps/cnpy/include")

add_subdirectory(deps/CMAPF)
set(CMAPF_INC "deps/CMAPF/include")

add_executable(${EXE} ${SOURCES})
target_include_directories(${EXE} PUBLIC ${INCLUDE_DIRS} ${CMAPF_INC} ${CNPY_INC})
target_link_libraries(${EXE} PRIVATE cnpy cmapf)

set(HEURISTIC MCA CACHE STRING "Choose the insertion heuristic")
message(Using heuristic ${HEURISTIC})
target_compile_definitions(${EXE} PUBLIC HEUR=${HEURISTIC})

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(${EXE} PRIVATE ${Boost_LIBRARIES})
