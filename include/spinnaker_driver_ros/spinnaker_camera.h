#pragma once

#include <mutex>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// ROS
#include "image_transport/image_transport.h"

/** @brief Class for Spinnaker cameras
 * TODO: Use configuration mutex
 */
class SpinnakerCamera {
 public:
  SpinnakerCamera();
  SpinnakerCamera(std::string serial, std::string id);
  ~SpinnakerCamera();

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

  /** @brief Starts the camera acquisition
   * @returns True if successful
   **/
  bool startAcquisition();

  /** @brief Stops the camera acquisition
   * @returns True if successful
   **/
  bool stopAcquisition();

  // Getters
  /** @brief Returns the Camera Serial Number
   * @returns Camera Serial Number
   **/
  std::string getSerial() const { return camera_serial; }

  /** @brief Get the Acquisition status
   * @return true if acquisition started
   * @return false if acquisition stopped
   */
  bool getAcquisition() const { return acquisition_started; }

  // Setters
  /** @brief Sets the save_image_flag
   * @param save_image 
   */
  void setSaveImage(bool save_image) { save_image_flag = save_image; }

  /** @brief Grabs any available image from the camera buffer
   * @param frame A pointer to an OpenCV Mat of the frame
   * @param file_name File name with full path to save the captured frame
   * @returns True if successful
   **/
  bool grabFrame(sensor_msgs::Image& frame, std::string& file_name);

 private:
  // Spinnaker handle for camera
  Spinnaker::CameraPtr camera_pointer;
  Spinnaker::GenApi::INodeMap* node_map;
  Spinnaker::GenApi::INodeMap* stream_node_map;
  Spinnaker::GenApi::INodeMap* device_node_map;

  // Camera parameters
  std::string camera_id;
  std::string camera_serial;
  bool acquisition_started;
  bool save_image_flag;
  std::unique_ptr<std::mutex> config_mutex;
};
