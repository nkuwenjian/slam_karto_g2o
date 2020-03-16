cmake_minimum_required(VERSION 2.8.3)
project(slam_karto_g2o)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

# Add new CMake Modules
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

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

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${SUITESPARSE_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/slam_karto_g2o.cpp src/g2o_solver.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES} 
)
