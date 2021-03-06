#pragma once

#include <mutex>
#include <thread>

// Spinnaker
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

// OpenCV
#include "opencv4/opencv2/opencv.hpp"

/** @brief Class for Spinnaker cameras
 */
class SpinnakerCamera {
 public:
  SpinnakerCamera();
  SpinnakerCamera(std::string serial, std::string id);
  ~SpinnakerCamera();

  /** @brief Establishes connection to the camera specified by the serial number
   * Also, it does basic camera initialization to allow SW and HW triggering.
   * It enables frame rate control, sets the auto exposure to off, auto gain to
   * off and the buffer handling to newest. If HW_trigger is true, then it
   * configures the cameras to receive trigger signals in line 0 and trigger at
   * the rising edge. If HW_trigger is false, then it sets the trigger to SW
   * @param camera_list List of available cameras
   * @param HW_trigger Enable HW trigger. If false, it enables SW triggers
   * @returns True if successful
   */
  bool connect(Spinnaker::CameraList camera_list, bool HW_trigger = true);

  /** @brief Terminates connection to the camera
   * @returns True if successful
   */
  bool disconnect();

  /** @brief Starts the camera acquisition
   * @returns True if successful
   */
  bool startAcquisition();

  /** @brief Stops the camera acquisition
   * @returns True if successful
   */
  bool stopAcquisition();

  /**
   * @brief Enables triggering for camera
   * @param enable If true enables triggering if false it disables it
   */
  void enableTriggering(bool enable);

  /**
   * @brief Executes a software trigger for the camera
   */
  void softwareTrigger();

  /** @brief Configures the camera.
   * @param exposure Exposure time
   * @param gain Gain
   * @param fps Frame rate
   * @returns True if successful
   */
  bool configure(double exposure, double gain, double fps);

  /** @brief Sets the camera exposure time
   * @param exposure Exposure time
   * @returns True if successful
   */
  bool setExposure(double exposure);

  /** @brief Sets the camera gain
   * @param gain Gain
   * @returns True if successful
   */
  bool setGain(double gain);

  /** @brief Sets the camera frame rate
   * @param fps Frame rate
   * @returns True if successful
   */
  bool setFPS(double fps);

  /** @brief Grabs any available image from the camera buffer
   * @param frame A pointer to an OpenCV Mat of the frame
   * @param delay How many milliseconds should it wait to grab the frame
   * @returns True if successful
   */
  bool grabFrame(cv::Mat& frame, uint64_t delay = 1000);

  /** @brief Returns the Camera Serial Number
   * @returns Camera Serial Number
   */
  std::string getSerial() const { return camera_serial; }

  /** @brief Get the Acquisition status
   * @returns true if acquisition started false if acquisition stopped
   */
  bool getAcquisition() const { return acquisition_started; }

  /**
   * @brief Get the min fps
   */
  double getFPSmin() const { return fps_min; }

  /**
   * @brief Get the max fps
   */
  double getFPSmax() const { return fps_max; }

  /**
   * @brief Get the min exposure time
   */
  double getExposureMin() const { return exp_min; }

  /**
   * @brief Get the max exposure time
   */
  double getExposureMax() const { return exp_max; }

  /**
   * @brief Get the min gain
   */
  double getGainMin() const { return gain_min; }

  /**
   * @brief Get the max gain
   */
  double getGainMax() const { return gain_max; }

 private:
  void getMinMaxValues();
  void getMinMaxConfigValues(double& min_v, double& max_v,
                             std::string config_name);

  // Spinnaker handle for camera
  Spinnaker::CameraPtr camera_pointer;
  Spinnaker::GenApi::INodeMap* node_map;
  Spinnaker::GenApi::INodeMap* stream_node_map;
  Spinnaker::GenApi::INodeMap* device_node_map;
  Spinnaker::GenApi::CCommandPtr software_trigger_ptr;

  // Camera parameters
  bool is_BFS, is_HW_trigger;
  double fps_max, fps_min, exp_max, exp_min, gain_max, gain_min;
  std::string camera_id;
  std::string camera_serial;
  bool acquisition_started;
};
