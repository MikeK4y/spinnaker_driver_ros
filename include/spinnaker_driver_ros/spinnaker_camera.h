#pragma once

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// OpenCV
#include "opencv2/opencv.hpp"

class SpinnakerCamera {
 public:
  SpinnakerCamera(std::string serial);
  ~SpinnakerCamera();

  /** @brief Establishes connection to the camera specified by the serial number
   * @param camera_list List of available cameras
   * @returns True if successful
   **/
  bool connect(Spinnaker::CameraList camera_list);

  /** @brief Terminates connection to the camera
   * @returns True if successful
   **/
  bool disconnect();

  /** @brief Configures the camera.
   * TODO: It only sets the Acquisition Mode to continuous. I need to add more
   * @returns True if successful
   **/
  bool configure();

  /** @brief Returns the Camera Serial Number
   * @returns Camera Serial Number
   **/
  std::string getSerial() const { return camera_serial; }

  /** @brief Grabs any available image from the camera buffer
   * @param frame A pointer to an OpenCV Mat of the frame
   * @param time_stamp A pointer to the time stamp in msec
   * @returns True if successful
   **/
  bool grabFrame(cv::Mat &frame, uint64_t &time_stamp);

  /** @brief Starts the camera acquisition
   * @returns True if successful
   **/
  bool startAcquisition();

  /** @brief Stops the camera acquisition
   * @returns True if successful
   **/
  bool stopAcquisition();

 private:
  // Spinnaker handle for camera
  Spinnaker::CameraPtr camera_pointer;
  Spinnaker::GenApi::INodeMap* node_map;
  Spinnaker::GenApi::INodeMap* stream_node_map;
  Spinnaker::GenApi::INodeMap* device_node_map;

  // Camera parameters
  std::string camera_serial;
  bool acquisition_started;
  uint32_t frame_counter;
};
