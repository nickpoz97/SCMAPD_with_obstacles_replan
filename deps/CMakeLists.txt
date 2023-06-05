cmake_minimum_required(VERSION 3.22)
project(dependencies)

# --- Dependencies manager ---
# Download dependencies here
set(CPM_SOURCE_CACHE
        "${CMAKE_BINARY_DIR}/cpm_sources"
        CACHE PATH "Directory to download CPM dependencies"
        FORCE)

# Include dependency manager
include(cmake/CPM.cmake)

# fmt dependency
CPMAddPackage(
        NAME fmt
        GITHUB_REPOSITORY fmtlib/fmt
        GIT_TAG 10.0.0
        EXCLUDE_FROM_ALL TRUE)

add_subdirectory(cnpy)
add_subdirectory(json)

find_package(Boost REQUIRED COMPONENTS program_options)

add_library(ExternalLibs INTERFACE)
target_link_libraries(ExternalLibs INTERFACE cnpy fmt::fmt nlohmann_json::nlohmann_json Boost::program_options pbs)
target_include_directories(
        ExternalLibs INTERFACE ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/cnpy/include ${CMAKE_CURRENT_SOURCE_DIR}/PBS/inc
)
