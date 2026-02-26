# This expects we build the core first, or that we have
# a "superbuild" that adds both.

# NOTE: we can later build it standalone, we can add the core
# as a subdirectory via a relative path
cmake_minimum_required(VERSION 3.16)
project(backyard_flyer_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED CXX)

# For a simple follow-along approach, include the core directly:
add_subdirectory(../backyard_flyer_core backyard_flyer_core_build)

add_executable(backyard_flyer_app
    src/main.cpp
)

target_link_libraries(backyard_flyer_app PRIVATE backyard_flyer_core)
