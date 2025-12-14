cmake_minimum_required(VERSION 3.15)
project(obstacle_cruise_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SOURCE_FILES
  app/main.cpp
  src/node.cpp
  src/planner_interface.cpp
  src/polygon_utils.cpp
  src/optimization_based_planner.cpp
)

add_executable(obstacle_cruise_planner ${SOURCE_FILES})
target_include_directories(
  obstacle_cruise_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/common
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ocp)
