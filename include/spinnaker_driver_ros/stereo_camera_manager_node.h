#pragma once

#include <functional>
#include <mutex>
#include <thread>
#include <future>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "camera_info_manager/camera_info_manager.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "spinnaker_driver_ros/stereoCameraParametersConfig.h"

// ROS messages
#include "sensor_msgs/image_encodings.h"

// SpinnakerCamera class
#include "spinnaker_camera.h"

class StereoCameraManagerNode {
 public:
  /** TODO: Add exceptions to stop constructor if something's not right */
  StereoCameraManagerNode(ros::NodeHandle &nh,
                          image_transport::ImageTransport &image_t);
  ~StereoCameraManagerNode();

  std::thread l_cam_worker, r_cam_worker;

 private:
  /** @brief Uses a private node handle to load the node parameters
   */
  void loadParameters();

  /** @brief Grabs the image from the camera and publishes it along with the
   * camera information TODO: Setup the camera info publisher and the config
   * mutex
   * @param camera The Spinnaker camera to grab the image from
   * @param image_pub The publisher for the image
   * @param camera_info_pub The publisher for the camera information
   * @param config_mutex The mutex for changing the configuration of the camera
   */
  void publishImage(SpinnakerCamera &camera,
                    image_transport::Publisher image_pub);

  /** @brief Callback for the dynamic reconfigure server
   */
  void dynamicReconfigureCallback(
      spinnaker_driver_ros::stereoCameraParametersConfig &config,
      uint32_t level);

  // Dynamic Reconfigure Server
  dynamic_reconfigure::Server<
      spinnaker_driver_ros::stereoCameraParametersConfig>
      config_server;

  // Spinnaker handles
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camera_list;

  // Camera parameters
  std::string l_cam_serial, r_cam_serial;
  SpinnakerCamera *l_camera, *r_camera;
  std::unique_ptr<std::mutex> l_cam_config_mutex, r_cam_config_mutex;
};
