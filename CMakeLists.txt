cmake_minimum_required(VERSION 3.0.2)
project(spinnaker_driver_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  sensor_msgs
)

# Spinnaker drivers for Ubuntu 20.04 need to be installed
# Available here: https://www.flir.com/products/spinnaker-sdk/
set(Spinnaker_INCLUDE_DIRS "/opt/spinnaker/include")
set(Spinnaker_LIBS "/opt/spinnaker/lib/libSpinnaker.so")

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/stereo_camera_parameters.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES spinnaker_driver_ros
 CATKIN_DEPENDS camera_info_manager cv_bridge dynamic_reconfigure image_transport roscpp rospy sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Spinnaker_INCLUDE_DIRS}
)

# Spinnaker camera
add_library(spinnaker_camera
  src/spinnaker_camera.cc
)

# Camera manager node
add_executable(camera_manager_node
  src/stereo_camera_manager_node.cc
  src/stereo_camera_manager_node_main.cc
)

## Dependencies
add_dependencies(camera_manager_node
  ${catkin_EXPORTED_TARGETS}
)

## Linking
target_link_libraries(spinnaker_camera
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBS}
)

target_link_libraries(camera_manager_node
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBS}
  spinnaker_camera
 )
