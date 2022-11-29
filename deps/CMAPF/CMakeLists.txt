cmake_minimum_required(VERSION 3.22)

project(CMAPF)
set(CMAKE_CXX_STANDARD 20)

file(GLOB_RECURSE SOURCES src/*.cpp)
set(INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)

add_library(cmapf ${SOURCES})

# --- Dependencies manager ---
# Download dependencies here
set(CPM_SOURCE_CACHE
        "${PROJECT_SOURCE_DIR}/deps/cpm_sources"
        CACHE PATH "Directory to download CPM dependencies"
        FORCE)

# Include dependency manager
include(cmake/CPM.cmake)

# fmt dependency
CPMAddPackage(
        NAME fmt
        GITHUB_REPOSITORY fmtlib/fmt
        GIT_TAG 9.1.0
        EXCLUDE_FROM_ALL TRUE)

target_include_directories(cmapf PRIVATE include)
target_link_libraries(cmapf PRIVATE fmt::fmt)


