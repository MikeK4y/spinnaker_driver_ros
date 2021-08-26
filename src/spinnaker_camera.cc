#include "spinnaker_driver_ros/spinnaker_camera.h"

#include "spinnaker_driver_ros/set_camera_feature.h"

#include <unistd.h>

SpinnakerCamera::SpinnakerCamera(std::string serial, std::string id)
    : acquisition_started(false), camera_serial(serial), camera_id(id) {
  save_images_flag = true;
  save_images_worker = std::thread(&SpinnakerCamera::saveImages, this);
}

SpinnakerCamera::~SpinnakerCamera() {
  while (!image_buffer.empty()) {
    std::cout << "Saving the images on the buffer\n";
    sleep(1);
  }
  save_images_flag = false;
  save_images_worker.join();

  camera_pointer->DeInit();
  delete camera_pointer;
}

bool SpinnakerCamera::connect(Spinnaker::CameraList camera_list) {
  try {
    camera_pointer = camera_list.GetBySerial(camera_serial);

  } catch (const Spinnaker::Exception& e) {
    std::cerr << "Could not find a camera with serial " << camera_serial
              << ". Error: " << e.what() << '\n';
    return false;
  }

  // Initialize Camera and get node maps
  try {
    camera_pointer->Init();
    node_map = &camera_pointer->GetNodeMap();
    stream_node_map = &camera_pointer->GetTLStreamNodeMap();
    device_node_map = &camera_pointer->GetTLDeviceNodeMap();

  } catch (const Spinnaker::Exception& e) {
    std::cerr << "Could not initialize camera. Error: " << e.what() << '\n';
    return false;
  }

  // Setup some basic configuration
  std::string stream_buffer_handling_mode = "NewestOnly";
  std::string acquisition_mode = "Continuous";
  std::string exposure_auto_mode = "Off";
  std::string gain_auto_mode = "Off";
  std::string trigger_source = "Line0";
  std::string trigger_activation = "RisingEdge";

  if (setFeature(stream_node_map, "StreamBufferHandlingMode",
                 stream_buffer_handling_mode))
    std::cout << "Stream buffer handling mode set to Newest Only\n";

  if (setFeature(node_map, "AcquisitionMode", acquisition_mode))
    std::cout << "Acquisition mode set to continuous\n";

  if (setFeature(node_map, "ExposureAuto", exposure_auto_mode))
    std::cout << "Auto Exposure set to off\n";

  if (setFeature(node_map, "GainAuto", gain_auto_mode))
    std::cout << "Auto Gain set to off\n";

  if (setFeature(node_map, "AcquisitionFrameRateEnable", true))
    std::cout << "Enabled Frame Rate Control\n";

  if (setFeature(node_map, "TriggerSource", trigger_source))
    std::cout << trigger_source << " was set as the trigger source\n";

  if (setFeature(node_map, "TriggerActivation", trigger_activation))
    std::cout << "Trigger activated on the " << trigger_activation << '\n';

  // Get min-max values for frame rate, exposure and gain
  getMinMaxValues();

  return camera_pointer->IsInitialized();
}

bool SpinnakerCamera::disconnect() {
  if (acquisition_started) stopAcquisition();

  try {
    if (camera_pointer->IsInitialized()) camera_pointer->DeInit();
    camera_pointer = nullptr;
  } catch (const std::exception& e) {
    std::cerr << "Could not disconnect camera. Error: " << e.what() << '\n';
    return false;
  }
  return true;
}

bool SpinnakerCamera::startAcquisition() {
  try {
    if (camera_pointer && !acquisition_started) {
      camera_pointer->BeginAcquisition();
      acquisition_started = true;
    }

  } catch (const std::exception& e) {
    std::cerr << "Could not start camera. Error: " << e.what() << '\n';
    return false;
  }

  return true;
}

bool SpinnakerCamera::stopAcquisition() {
  try {
    if (camera_pointer && acquisition_started) {
      camera_pointer->EndAcquisition();
      acquisition_started = false;
    }

  } catch (const std::exception& e) {
    std::cerr << "Could not stop camera. Error: " << e.what() << '\n';
    return false;
  }

  return true;
}

void SpinnakerCamera::setHardwareTrigger() {
  std::string trigger_mode = "On";

  if (setFeature(node_map, "TriggerMode", trigger_mode))
    std::cout << "Enabled triggering\n";
}

void SpinnakerCamera::setContinuousCapture() {
  std::string trigger_mode = "Off";

  if (setFeature(node_map, "TriggerMode", trigger_mode))
    std::cout << "Enabled continuous Acquisition\n";
}

bool SpinnakerCamera::configure(double exposure, double gain, double fps) {
  if (setFeature(node_map, "ExposureTime", exposure))
    std::cout << "Exposure time set to " << exposure << "\n";

  if (setFeature(node_map, "Gain", gain))
    std::cout << "Gain set to " << gain << "\n";

  if (setFeature(node_map, "AcquisitionFrameRate", fps))
    std::cout << "Frame rate set to " << fps << "fps\n";

  return true;
}

bool SpinnakerCamera::setExposure(double exposure) {
  return setFeature(node_map, "ExposureTime", exposure);
}

bool SpinnakerCamera::setGain(double gain) {
  return setFeature(node_map, "Gain", gain);
}

bool SpinnakerCamera::setFPS(double fps) {
  return setFeature(node_map, "AcquisitionFrameRate", fps);
}

bool SpinnakerCamera::grabFrame(cv::Mat& frame, std::string& file_name,
                                uint64_t delay, bool save_frame) {
  if (acquisition_started) {
    try {
      Spinnaker::ImagePtr spin_image_raw = camera_pointer->GetNextImage(delay);

      if (spin_image_raw->IsIncomplete()) {
        std::cerr << "Image was incomplete" << '\n';
        return false;
      } else {
        // Make sure the pixel depth is 8bit
        size_t bits_per_pixel = spin_image_raw->GetBitsPerPixel();

        if (bits_per_pixel == 8) {
          // Save Image
          if (save_frame) {
            std::lock_guard<std::mutex> image_buffer_guard(*image_buffer_mutex);
            image_buffer.push_back(spin_image_raw);
            image_name_buffer.push_back(file_name);
          }

          // Convert to CV Mat
          frame = cv::Mat(
              spin_image_raw->GetHeight() + spin_image_raw->GetYPadding(),
              spin_image_raw->GetWidth() + spin_image_raw->GetXPadding(),
              CV_8UC1, spin_image_raw->GetData(), spin_image_raw->GetStride());
        } else {
          std::cerr << "Image depth = " << bits_per_pixel << " bits.\n";
          return false;
        }
      }
    } catch (const Spinnaker::Exception& e) {
      // std::cerr << "Could not grab frame. Error: " << e.what() << '\n';
      return false;
    }
  } else {
    std::cerr << "Camera acquisition has not started yet\n";
    return false;
  }
  return true;
}

void SpinnakerCamera::saveImages() {
  while (true) {
    {  // Anonymous to release mutex in every loop
      std::lock_guard<std::mutex> image_buffer_guard(*image_buffer_mutex);

      if (image_buffer.size() > 0) {
        image_buffer.front()->Save(image_name_buffer.front().c_str());
        image_buffer.pop_front();
        image_name_buffer.pop_front();
      }
    }
  }
}

void SpinnakerCamera::getMinMaxValues() {
  fps_max, fps_min, exp_max, exp_min, gain_max, gain_min = -1;
  getMinMaxConfigValues(fps_min, fps_max, "AcquisitionFrameRate");
  getMinMaxConfigValues(exp_min, exp_max, "ExposureTime");
  getMinMaxConfigValues(gain_min, gain_max, "Gain");
}

void SpinnakerCamera::getMinMaxConfigValues(double& min_v, double& max_v,
                                            std::string config_name) {
  Spinnaker::GenApi::CFloatPtr feature_ptr =
      node_map->GetNode(config_name.c_str());

  if (!Spinnaker::GenApi::IsAvailable(feature_ptr) ||
      !Spinnaker::GenApi::IsWritable(feature_ptr)) {
    std::cerr << config_name << " is not Available/Writable\n";
  }

  min_v = feature_ptr->GetMin();
  max_v = feature_ptr->GetMax();
}