cmake_minimum_required(VERSION 3.0.2)
project(spinnaker_driver_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

## Find OpenCV
find_package(OpenCV REQUIRED)

# Spinnaker drivers for Ubuntu 20.04 need to be installed
# Available here: https://www.flir.com/products/spinnaker-sdk/
set(Spinnaker_INCLUDE_DIRS "/opt/spinnaker/include")
set(Spinnaker_LIBS "/opt/spinnaker/lib/libSpinnaker.so")

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES spinnaker_driver_ros
 CATKIN_DEPENDS camera_info_manager cv_bridge image_transport roscpp sensor_msgs
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
  ${Spinnaker_LIBS}
  ${OpenCV_LIBS}
)

target_link_libraries(camera_manager_node
  ${catkin_LIBRARIES}
  ${Spinnaker_LIBS}
  ${OpenCV_LIBS}
  spinnaker_camera
 )
