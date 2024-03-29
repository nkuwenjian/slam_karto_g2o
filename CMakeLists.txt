cmake_minimum_required(VERSION 3.0.2)
project(slam_karto_g2o)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra")

# Add new CMake Modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/slam_karto_g2o;
#   -warnings-as-errors=*;
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  tf
  visualization_msgs
  open_karto
)

# Eigen
find_package(Eigen3 REQUIRED)

# g2o
find_package(G2O REQUIRED)

# suitesparse
find_package(SUITESPARSE REQUIRED)

find_package(glog REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)

add_library(g2o_solver slam_karto_g2o/g2o_solver.cc)
target_link_libraries(g2o_solver
  ${catkin_LIBRARIES}
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  Eigen3::Eigen
  glog::glog
)

add_executable(slam_karto_g2o
  slam_karto_g2o/slam_karto_g2o.cc
  slam_karto_g2o/slam_karto_g2o_node.cc
)
target_link_libraries(slam_karto_g2o g2o_solver ${catkin_LIBRARIES})

add_executable(slam_karto_g2o_test test/slam_karto_g2o_test.cc)
target_link_libraries(slam_karto_g2o_test g2o_solver)
