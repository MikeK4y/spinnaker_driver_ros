#pragma once

#include <fstream>
#include <future>
#include <thread>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"

// ROS messages
#include "mavros_msgs/CamIMUStamp.h"
#include "sensor_msgs/image_encodings.h"

// SpinnakerCamera class
#include "spinnaker_camera.h"

namespace spinnaker_driver_ros {

class SyncedStereoNodelet : public nodelet::Nodelet {
 public:
  /** TODO: Add exceptions to stop constructor if something's not right */
  SyncedStereoNodelet() {}
  ~SyncedStereoNodelet();

 private:
  virtual void onInit();

  /** @brief Uses a private node handle to load the node parameters
   */
  void loadParameters();

  /** @brief Grabs the two images in parallel trying to sync the cameras
   */
  void publishImagesSync();

  /**
   * @brief Use MavROS Service to control camera hardware triggering
   * @param enable Enable or disable trigger
   * @param reset Reset sequence
   * @returns True if successful
   */
  bool triggerControl(bool enable, bool reset);

  /**
   * @brief Use MavROS Service to configure hardware triggering
   * @param fps Trigger rate
   * @returns True if successful
   */
  bool triggerConfig(double fps);

  /**
   * @brief Callback for the trigger timepstamps
   * @param msg MavROS timestamp message
   */
  void triggerStampCallback(const mavros_msgs::CamIMUStamp &msg);

  /**
   * @brief Find the timestamp for the frame
   * @param frame_index Frame index
   * @returns timestamp
   */
  bool getTimestamp(uint32_t frame_index, ros::Time &timestamp);

  // ROS Publishers
  image_transport::Publisher l_image_pub, r_image_pub;
  ros::Publisher l_cam_info_pub, r_cam_info_pub;

  // ROS Subscribers
  ros::Subscriber trigger_time_stamp_sub;

  // ROS Services
  ros::ServiceClient pixhawk_trigger_ctrl, pixhawk_trigger_config;

  // Spinnaker handles
  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camera_list;

  // Camera parameters
  SpinnakerCamera *l_camera, *r_camera;
  sensor_msgs::CameraInfo l_cam_info, r_cam_info;
  std::string l_cam_serial, r_cam_serial;
  double fps, exp, gain;
  std::thread frame_grab_worker;

  // Image parameters
  std::vector<mavros_msgs::CamIMUStamp> timestamp_buffer;
  size_t buffer_size;
  uint32_t frame_count;
};

}  // namespace spinnaker_driver_ros