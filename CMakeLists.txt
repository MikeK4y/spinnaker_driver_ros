cmake_minimum_required(VERSION 3.0.2)
project(spinnaker_driver_ros)

add_compile_options(-std=c++14)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_transport
  mavros
  nodelet
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV 4 REQUIRED)

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
 CATKIN_DEPENDS camera_info_manager cv_bridge dynamic_reconfigure image_transport mavros nodelet roscpp rospy sensor_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Spinnaker_INCLUDE_DIRS}
)

# Spinnaker camera library
add_library(spinnaker_camera
  src/spinnaker_camera.cc
)

# SW trigger nodelet library
add_library(sw_trigger_nodelet
  src/sw_trigger_nodelet.cc
)

# SW trigger nodelet runnable
add_executable(sw_trigger_nodelet_runnable
  src/sw_trigger_nodelet_main.cc
)

# HW trigger nodelet library
add_library(hw_trigger_nodelet
  src/hw_trigger_nodelet.cc
)

# HW trigger nodelet runnable
add_executable(hw_trigger_nodelet_runnable
  src/hw_trigger_nodelet_main.cc
)

##################
## Dependencies ##
##################

add_dependencies(sw_trigger_nodelet
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(hw_trigger_nodelet
  ${catkin_EXPORTED_TARGETS}
)

#############
## Linking ##
#############

target_link_libraries(spinnaker_camera
  ${OpenCV_LIBS}
  ${Spinnaker_LIBS}
)

target_link_libraries(sw_trigger_nodelet
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBS}
  spinnaker_camera
)

target_link_libraries(sw_trigger_nodelet_runnable
sw_trigger_nodelet
)

target_link_libraries(hw_trigger_nodelet
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBS}
  spinnaker_camera
)

target_link_libraries(hw_trigger_nodelet_runnable
hw_trigger_nodelet
)

#############
## Install ##
#############

install(FILES
  sw_trigger_nodelet_plugins.xml
  hw_trigger_nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  sw_trigger_nodelet_runnable
  hw_trigger_nodelet_runnable
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
