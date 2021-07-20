#pragma once

#include <fstream>
#include <future>
#include <mutex>
#include <thread>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include <dynamic_reconfigure/server.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

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
  virtual ~StereoCameraManagerNodelet();

  std::thread frame_grab_worker;

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
  ros::Publisher l_cam_info_pub, r_cam_info_pub;

  // Spinnaker handles
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camera_list;

  // Camera parameters
  std::string l_cam_serial, r_cam_serial;
  SpinnakerCamera *l_camera, *r_camera;
  sensor_msgs::CameraInfo l_cam_info, r_cam_info;
  sensor_msgs::CameraInfo l_cam_info_resized, r_cam_info_resized;
  std::unique_ptr<std::mutex> config_mutex;
  spinnaker_driver_ros::stereoCameraParametersConfig current_config;

  // Image folder
  std::string path_to_images;
  std::ofstream image_list_file;
  uint64_t frame_count;
  uint64_t saved_frame_count;
  uint64_t save_percent;
  double resize_factor;
  bool save_images;
  bool resize_images;
  ros::Time startTime;
};

}  // namespace spinnaker_driver_ros