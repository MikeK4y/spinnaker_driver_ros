#pragma once

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// OpenCV
#include "opencv2/opencv.hpp"

class SpinnakerCamera {
 public:
  SpinnakerCamera();
  SpinnakerCamera(std::string serial);
  ~SpinnakerCamera();

  /** @brief Sets the serial number of the camera
   * @param serial A string with the camera's serial number
   */
  void setSerial(std::string serial) { camera_serial = serial; }

  /** @brief Establishes connection to the camera specified by the serial number
   * Also, it initializes the camera by setting the Auto Exposure to Off,
   * the buffer handling to newest, acquisition mode to continuous and
   * enbale frame rate control
   * @param camera_list List of available cameras
   * @returns True if successful
   **/
  bool connect(Spinnaker::CameraList camera_list);

  /** @brief Terminates connection to the camera
   * @returns True if successful
   **/
  bool disconnect();

  /** @brief Configures the camera.
   * @param exposure Exposure time
   * @param fps Frame rate
   * @returns True if successful
   **/
  bool configure(double exposure, double fps);

  /** @brief Returns the Camera Serial Number
   * @returns Camera Serial Number
   **/
  std::string getSerial() const { return camera_serial; }

  /** @brief Grabs any available image from the camera buffer
   * TODO: Instead of using an OpenCV Mat for the image use a sensor_msgs::Image
   * TODO: Pass a string with the path to save the image
   * @param frame A pointer to an OpenCV Mat of the frame
   * @param time_stamp A pointer to the time stamp in msec
   * @returns True if successful
   **/
  bool grabFrame(cv::Mat& frame, uint64_t& time_stamp);

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
