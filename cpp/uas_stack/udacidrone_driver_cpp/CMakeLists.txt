cmake_minimum_required(VERSION 3.8)
project(udacidrone_driver_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(libmavconn REQUIRED)

add_executable(uas_driver_node
  src/main.cpp
  src/simulator_connection.cpp
  src/mavlink_translator.cpp
  src/uas_state_repository.cpp
  src/uas_command_service.cpp
  src/driver_session_manager.cpp
  src/command_gatekeeper.cpp
  src/driver_health_monitor.cpp
  src/uas_driver_node.cpp
)

target_include_directories(uas_driver_node PRIVATE include)

ament_target_dependencies(uas_driver_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  mavlink
  mavros_msgs
  libmavconn
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS uas_driver_node DESTINATION lib/${PROJECT_NAME})

ament_package()
