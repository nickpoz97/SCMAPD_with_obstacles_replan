project("Simultaneous CMAPD")

cmake_minimum_required(VERSION 3.12)

set(EXE cmapd)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(${EXE} ${SOURCES})
