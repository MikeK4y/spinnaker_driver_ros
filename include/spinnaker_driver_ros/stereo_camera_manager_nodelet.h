#pragma once

#include <future>
#include <mutex>
#include <thread>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include "dynamic_reconfigure/server.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"
#include "spinnaker_driver_ros/stereoCameraParametersConfig.h"

// ROS messages
#include "sensor_msgs/image_encodings.h"

// SpinnakerCamera class
#include "spinnaker_camera.h"

namespace spinnaker_driver_ros {

class StereoCameraManagerNodelet : public nodelet::Nodelet {
 public:
  /** TODO: Add exceptions to stop constructor if something's not right */
  StereoCameraManagerNodelet() {}
  ~StereoCameraManagerNodelet();

 private:
  virtual void onInit();

  /** @brief Uses a private node handle to load the node parameters
   */
  void loadParameters();

  /** @brief Grabs the two images in parallel trying to sync the cameras
   */
  void publishImagesSync();

  /** @brief Callback for the dynamic reconfigure server
   */
  void dynamicReconfigureCallback(
      spinnaker_driver_ros::stereoCameraParametersConfig &config,
      uint32_t level);

  // Dynamic Reconfigure Server
  dynamic_reconfigure::Server<
      spinnaker_driver_ros::stereoCameraParametersConfig>
      config_server;

  // Publishers
  image_transport::Publisher l_image_pub, r_image_pub;
  ros::Publisher l_cam_info_pub, r_cam_info_pub, cam_exp_pub, cam_gain_pub;

  // Spinnaker handles
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camera_list;

  // Camera parameters
  SpinnakerCamera *l_camera, *r_camera;
  sensor_msgs::CameraInfo l_cam_info, r_cam_info;
  sensor_msgs::CameraInfo l_cam_info_resized, r_cam_info_resized;
  std::string l_cam_serial, r_cam_serial;
  std::unique_ptr<std::mutex> config_mutex;
  spinnaker_driver_ros::stereoCameraParametersConfig current_config;
  std::thread frame_grab_worker;
  uint64_t frame_count;
  ros::Rate *frame_rate;
};

}  // namespace spinnaker_driver_ros