cmake_minimum_required(VERSION 3.0.2)
project(bolt_imu)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/share/c++-mscl/source
#   /usr/share/c++-mscl/Boost/include
)

link_directories(
      /usr/share/c++-mscl
)
add_library(mx5_lib
  src/mx5_imu.cpp  
)
add_executable(${PROJECT_NAME} src/sensor_manager.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mx5_lib
  -lmscl  
#   /usr/share/c++-mscl/Boost/lib/libboost_system.so.1.68.0
)

add_dependencies(mx5_lib ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})