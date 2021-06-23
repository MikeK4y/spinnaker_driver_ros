#pragma once

#include <functional>
#include <mutex>
#include <thread>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include <ros/ros.h>

#include "camera_info_manager/camera_info_manager.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

// ROS messages
#include "sensor_msgs/image_encodings.h"

// SpinnakerCamera class
#include "spinnaker_camera.h"

class StereoCameraManagerNode {
 public:
  StereoCameraManagerNode(ros::NodeHandle &nh,
                          image_transport::ImageTransport &image_t);
  ~StereoCameraManagerNode();

  std::thread l_cam_worker, r_cam_worker;

 private:
  /** @brief Uses a private node handle to load the node parameters
   */
  void loadParameters();

  /** @brief Grabs the image from the camera and publishes it along with the
   * camera information
   * @param camera The Spinnaker camera to grab the image from
   * @param image_pub The publisher for the image
   * @param camera_info_pub The publisher for the camera information
   * @param config_mutex The mutex for changing the configuration of the camera
   */
  void publishImage(SpinnakerCamera &camera,
                    image_transport::Publisher image_pub);
  // ros::Publisher camera_info_pub);

  // Spinnaker handles
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camera_list;

  // Camera parameters
  std::string l_cam_serial, r_cam_serial;
  SpinnakerCamera *l_camera, *r_camera;
  std::unique_ptr<std::mutex> l_cam_config_mutex, r_cam_config_mutex;
};
