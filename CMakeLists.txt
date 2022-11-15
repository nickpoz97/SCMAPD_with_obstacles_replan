project("Simultaneous CMAPD")

cmake_minimum_required(VERSION 3.23)

set(CMAKE_CXX_STANDARD 17)

set(EXE cmapd)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

add_executable(${EXE} ${SOURCES})
target_include_directories(${EXE} PUBLIC ${INCLUDE_DIRS})

add_subdirectory(deps/cnpy)
file(GLOB CNPY_INC "deps/cnpy/inc")
target_include_directories(${EXE} PUBLIC ${CNPY_INC})
target_link_libraries(${EXE} PRIVATE cnpy)

add_subdirectory(deps/PBS)
file(GLOB PBS_INC "deps/PBS/inc")
target_include_directories(${EXE} PUBLIC ${PBS_INC})
target_link_libraries(${EXE} PRIVATE pbs)

find_package(ortools CONFIG REQUIRED)
target_link_libraries(${EXE} PRIVATE ortools)
target_include_directories(${EXE} PUBLIC ${ortools_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(${EXE} PRIVATE ${Boost_LIBRARIES})
