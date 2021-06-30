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
#include <ros/ros.h>

#include "camera_info_manager/camera_info_manager.h"
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

  std::thread frame_grab_worker;

 private:
  /** @brief Uses a private node handle to load the node parameters
   */
  void loadParameters();

  /** @brief Grabs the two images in parallel trying to sync the cameras
   * TODO: Setup the camera info publisher
   * TODO: I need to check that both images were captured successfully before
   * doing anything
   * @param left_camera The left Spinnaker camera in the stereo pair
   * @param right_camera The right Spinnaker camera in the stereo pair
   * @param left_image_pub Left image publisher
   * @param right_image_pub Right image publisher
   */
  void publishImagesSync(SpinnakerCamera &left_camera,
                         SpinnakerCamera &right_camera,
                         image_transport::Publisher left_image_pub,
                         image_transport::Publisher right_image_pub);

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
  std::unique_ptr<std::mutex> config_mutex;

  // Image folder
  std::string path_to_images;
  std::ofstream image_list_file;
  uint64_t frame_count;
  bool save_images;
  ros::Time startTime;
};
