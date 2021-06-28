#pragma once

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include "image_transport/image_transport.h"

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
   * Also, it initializes the camera by setting the auto exposure to off, auto
   * gain to off, the buffer handling to newest, acquisition mode to continuous
   * and enbale frame rate control
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
   * @param gain Gain
   * @param fps Frame rate
   * @returns True if successful
   **/
  bool configure(double exposure, double gain, double fps);

  /** @brief Returns the Camera Serial Number
   * @returns Camera Serial Number
   **/
  std::string getSerial() const { return camera_serial; }

  /** @brief Grabs any available image from the camera buffer
   * @param frame A pointer to an OpenCV Mat of the frame
   * @param file_name File name with full path to save the captured frame
   * @returns True if successful
   **/
  bool grabFrame(sensor_msgs::Image& frame, std::string& file_name);

  /** @brief Starts the camera acquisition
   * @returns True if successful
   **/
  bool startAcquisition();

  /** @brief Stops the camera acquisition
   * @returns True if successful
   **/
  bool stopAcquisition();

  bool getAcquisition() const { return acquisition_started; }

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
