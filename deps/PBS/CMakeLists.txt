project(pbs)

cmake_minimum_required (VERSION 2.6)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_library(pbs ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(pbs ${Boost_LIBRARIES})

if(MSVC)
    set_target_properties(
        pbs
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()
