cmake_minimum_required(VERSION 3.0.2)
project(franka_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp rosconsole roslib
  rospy
  std_msgs
  franka_hw
)
find_package(Eigen3 REQUIRED)

find_package(Franka 0.7.0 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_simulation
  CATKIN_DEPENDS roscpp std_msgs roslib rosconsole roscpp
  DEPENDS roslib rosconsole roscpp Franka franka_hw
)

set(${PROJECT_NAME}_headers
  include/franka_simulation/model_sim.h
  include/franka_simulation/franka_model_interface_sim.h
  include/franka_simulation/franka_model.h
)

set(${PROJECT_NAME}_sources
    src/model_sim.cpp
    src/GravityVector.cpp
    src/FrictionTorque.cpp
    #src/CoriolisMatrix.cpp
    src/MassMatrix.cpp
)

link_directories("/opt/ros/melodic/lib")

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(franka_simulation ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})

add_dependencies(franka_simulation ${catkin_EXPORTED_TARGETS})
target_link_libraries(franka_simulation
    roslib
    rosconsole
    rostime
    rospack
    roscpp
   ${catkin_LIBRARIES}
   ${Franka_LIBRARIES}
   ${franka_hw_LIBRARIES}
)

## Installation
#install(TARGETS franka_simulation
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

