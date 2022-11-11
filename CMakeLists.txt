project("Simultaneous CMAPD")

cmake_minimum_required(VERSION 3.12)

set(EXE cmapd)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

add_executable(${EXE} ${SOURCES})
target_include_directories(${EXE} PUBLIC ${INCLUDE_DIRS})
