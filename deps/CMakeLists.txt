cmake_minimum_required(VERSION 3.22)
project(dependencies)

# --- Dependencies manager ---
# Download dependencies here
set(CPM_SOURCE_CACHE
        "${PROJECT_SOURCE_DIR}/cpm_sources"
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

add_subdirectory(cnpy)
add_subdirectory(json)
