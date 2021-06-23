#include "spinnaker_driver_ros/spinnaker_camera.h"

#include "spinnaker_driver_ros/set_camera_feature.h"

SpinnakerCamera::SpinnakerCamera(std::string serial)
    : acquisition_started(false), camera_serial(serial), frame_counter(0) {}

SpinnakerCamera::~SpinnakerCamera() {}

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

bool SpinnakerCamera::configure() {
  std::string stream_buffer_handling_mode = "NewestOnly";
  std::string acquisition_mode = "Continuous";
  std::string exposure_auto_mode = "Off";
  double exposure = 500.0;
  double fps = 2.0;

  if (setFeature(stream_node_map, "StreamBufferHandlingMode",
                 stream_buffer_handling_mode))
    std::cout << "Stream buffer handling mode set to Newest Only\n";

  if (setFeature(node_map, "AcquisitionMode", acquisition_mode))
    std::cout << "Acquisition mode set to continuous\n";

  if (setFeature(node_map, "ExposureAuto", exposure_auto_mode))
    std::cout << "Auto Exposure set to off\n";

  if (setFeature(node_map, "ExposureTime", exposure))
    std::cout << "Exposure time set to " << exposure << "\n";

  if (setFeature(node_map, "AcquisitionFrameRateEnable", true))
    std::cout << "Enabled Frame Rate Control\n";

  if (setFeature(node_map, "AcquisitionFrameRate", fps))
    std::cout << "Frame rate set to " << fps << "fps\n";

  return true;
}

bool SpinnakerCamera::grabFrame(cv::Mat& frame, uint64_t& time_stamp) {
  if (acquisition_started) {
    try {
      Spinnaker::ImagePtr spin_image_raw = camera_pointer->GetNextImage(1000);

      if (spin_image_raw->IsIncomplete()) {
        std::cerr << "Image was incomplete" << '\n';
        return false;
      } else {
        // Get timestamp and make sure the pixel depth is 8bit
        time_stamp = spin_image_raw->GetTimeStamp();
        size_t bits_per_pixel = spin_image_raw->GetBitsPerPixel();

        if (bits_per_pixel == 8) {
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
      std::cerr << "Could not grab frame. Error: " << e.what() << '\n';
      return false;
    }
  } else {
    std::cerr << "Camera acquisition has not started yet\n";
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
