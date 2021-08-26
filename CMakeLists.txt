cmake_minimum_required(VERSION 3.0.2)
project(spinnaker_driver_ros)

add_compile_options(-std=c++14)
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

# Camera manager nodelet library
add_library(camera_manager_nodelet
  src/stereo_camera_manager_nodelet.cc
)

# Camera manager nodelet runnable
add_executable(camera_manager_nodelet_runnable
  src/stereo_camera_manager_nodelet_main.cc
)

##################
## Dependencies ##
##################

add_dependencies(camera_manager_nodelet
  ${catkin_EXPORTED_TARGETS}
)

#############
## Linking ##
#############

target_link_libraries(spinnaker_camera
  ${OpenCV_LIBS}
  ${Spinnaker_LIBS}
)

target_link_libraries(camera_manager_nodelet
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBS}
  spinnaker_camera
)

target_link_libraries(camera_manager_nodelet_runnable
  camera_manager_nodelet
)

#############
## Install ##
#############

install(FILES stereo_camera_manager_nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY cfg launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  # camera_manager_node
  camera_manager_nodelet_runnable
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)