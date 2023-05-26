cmake_minimum_required(VERSION 3.22)
project(Simultaneous_CMAPD)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
endif()

set(SCMAPD_EXE scmapd)
set(OBS_EXE obstacle_avoidance)

file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)
list(FILTER SOURCES EXCLUDE REGEX "${PROJECT_SOURCE_DIR}/src/main/.*\\.cpp$")

set(INCLUDE_DIRS include deps/cnpy/include)

add_subdirectory(deps)

add_executable(${SCMAPD_EXE} ${SOURCES} src/main/scmapdMain.cpp)
add_executable(${OBS_EXE} ${SOURCES} src/main/obsAvoid.cpp)

find_package(Boost REQUIRED COMPONENTS program_options)

target_include_directories(${SCMAPD_EXE} PUBLIC ${INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(${SCMAPD_EXE} PRIVATE cnpy fmt::fmt nlohmann_json::nlohmann_json Boost::program_options pbs)

target_include_directories(${OBS_EXE} PUBLIC ${INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(${OBS_EXE} PRIVATE cnpy fmt::fmt nlohmann_json::nlohmann_json Boost::program_options)

set(CMAKE_CXX_FLAGS_SANIT "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address,leak,undefined -fno-omit-frame-pointer" )

message("Debug flags: ${CMAKE_CXX_FLAGS_DEBUG} -O0 -DNDEBUG")
message("Sanitizer flags: ${CMAKE_CXX_FLAGS_DEBUG}")
message("Release flags: ${CMAKE_CXX_FLAGS_RELEASE}")
message("RelWithDebInfo flags: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR} CACHE PATH "default installation path" FORCE)
set(INSTALL_DIR_NAME scmapd)

install(TARGETS ${SCMAPD_EXE} DESTINATION ${INSTALL_DIR_NAME})
install(TARGETS ${OBS_EXE} DESTINATION ${INSTALL_DIR_NAME})
install(DIRECTORY data DESTINATION ${INSTALL_DIR_NAME})

file(GLOB_RECURSE SCRIPTS scripts/*.py)
install(PROGRAMS ${SCRIPTS} DESTINATION ${INSTALL_DIR_NAME})
