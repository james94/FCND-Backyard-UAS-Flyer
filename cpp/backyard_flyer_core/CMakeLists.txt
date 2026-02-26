cmake_minimum_required(VERSION 3.16)
project(backyard_flyer_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(backyard_flyer_core
    src/planner.cpp
    src/controller.cpp
)

target_include_directories(backyard_flyer_core PUBLIC include)

option(BY_BUILD_TESTS "Build tests" ON)
if(BY_BUILD_TESTS)
    enable_testing()

    # Option A: bring your own doctest header in tests/third_party/doctest/doctest.h
    add_executable(test_planner tests/test_planner.cpp)
    target_link_libraries(test_planner PRIVATE backyard_flyer_core)
    add_test(NAME test_planner COMMAND test_planner)

    add_executable(test_guards tests/test_guards.cpp)
    target_link_libraries(test_guards PRIVATE backyard_flyer_core)
    add_test(NAME test_guards COMMAND test_guards)

    add_executable(test_controller tests/test_controller.cpp)
    target_link_libraries(test_controller PRIVATE backyard_flyer_core)
    add_test(NAME test_controller COMMAND test_controller)
endif()
