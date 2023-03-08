cmake_minimum_required(VERSION 3.22)
project(Simultaneous_CMAPD)

set(CMAKE_CXX_STANDARD 20)

set(EXE scmapd)

file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
set(INCLUDE_DIRS include)

add_subdirectory(deps)
set(CNPY_INCLUDE_DIR "deps/cnpy/include")

add_executable(${EXE} ${SOURCES})
target_include_directories(${EXE} PUBLIC ${INCLUDE_DIRS} ${CNPY_INCLUDE_DIR})
target_link_libraries(${EXE} PRIVATE cnpy fmt::fmt nlohmann_json::nlohmann_json)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(${EXE} PRIVATE ${Boost_LIBRARIES})

set(CMAKE_CXX_FLAGS_SANIT "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,leak,undefined -fno-omit-frame-pointer" )
message("Debug flags: ${CMAKE_CXX_FLAGS_DEBUG} -O0")
message("Sanitizer flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message("Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")
message("RelWithDebInfo flags: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR} CACHE PATH "default installation path" FORCE)

set(INSTALL_DIR_NAME scmapd)

install(TARGETS ${EXE} DESTINATION ${INSTALL_DIR_NAME})
install(DIRECTORY data DESTINATION ${INSTALL_DIR_NAME})

file(GLOB_RECURSE SCRIPTS scripts/*)
install(PROGRAMS ${SCRIPTS} DESTINATION ${INSTALL_DIR_NAME})
