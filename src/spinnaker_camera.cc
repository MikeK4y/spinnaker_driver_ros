#include "spinnaker_driver_ros/spinnaker_camera.h"

#include "spinnaker_driver_ros/set_camera_feature.h"

SpinnakerCamera::SpinnakerCamera(std::string serial, std::string id)
    : acquisition_started(false),
      camera_serial(serial),
      camera_id(id),
      save_image_flag(false) {}

SpinnakerCamera::~SpinnakerCamera() {
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

bool SpinnakerCamera::configure(double exposure, double gain, double fps) {
  if (setFeature(node_map, "ExposureTime", exposure))
    std::cout << "Exposure time set to " << exposure << "\n";

  if (setFeature(node_map, "Gain", gain))
    std::cout << "Gain set to " << gain << "\n";

  if (setFeature(node_map, "AcquisitionFrameRate", fps))
    std::cout << "Frame rate set to " << fps << "fps\n";

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

bool SpinnakerCamera::grabFrame(sensor_msgs::Image& frame,
                                std::string& file_name) {
  if (acquisition_started) {
    try {
      Spinnaker::ImagePtr spin_image_raw = camera_pointer->GetNextImage(1000);
      ros::Time timestamp = ros::Time::now();

      if (spin_image_raw->IsIncomplete()) {
        std::cerr << "Image was incomplete" << '\n';
        return false;
      } else {
        // Make sure the pixel depth is 8bit
        size_t bits_per_pixel = spin_image_raw->GetBitsPerPixel();

        if (bits_per_pixel == 8) {
          // Save Image
          if (save_image_flag) spin_image_raw->Save(file_name.c_str());

          // Convert to sensor_msg::Image
          frame.width =
              spin_image_raw->GetWidth() + spin_image_raw->GetXPadding();
          frame.height =
              spin_image_raw->GetHeight() + spin_image_raw->GetYPadding();
          frame.step = spin_image_raw->GetStride();
          frame.encoding = "mono8";
          size_t data_size = frame.height * frame.step;
          frame.data.resize(data_size);
          memcpy(&frame.data[0], spin_image_raw->GetData(), data_size);

          // Prepare header
          frame.header.stamp = timestamp;
          frame.header.frame_id = camera_id;
        } else {
          std::cerr << "Image depth = " << bits_per_pixel << " bits.\n";
          return false;
        }
      }
    } catch (const Spinnaker::Exception& e) {
      std::cerr << "Could not grab frame. Error: " << e.what() << '\n';
      return false;
    }
  } else {
    std::cerr << "Camera acquisition has not started yet\n";
    return false;
  }
  return true;
}
