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
