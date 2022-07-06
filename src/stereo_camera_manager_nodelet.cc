#include "spinnaker_driver_ros/stereo_camera_manager_nodelet.h"

#include <functional>

// ROS
#include <pluginlib/class_list_macros.h>

#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Float32.h"

PLUGINLIB_EXPORT_CLASS(spinnaker_driver_ros::StereoCameraManagerNodelet,
                       nodelet::Nodelet);

inline sensor_msgs::Image toROSImageMsg(cv::Mat frame, ros::Time timestamp) {
  cv_bridge::CvImage ros_mat;
  sensor_msgs::Image ros_image;

  ros_mat.image = frame;
  ros_mat.header.stamp = timestamp;
  ros_mat.encoding = "mono8";

  ros_mat.toImageMsg(ros_image);

  return ros_image;
}

namespace spinnaker_driver_ros {

void StereoCameraManagerNodelet::onInit() {
  ros::NodeHandle nh = getNodeHandle();
  image_transport::ImageTransport image_t(nh);
  // Initialize Spinnaker handles
  system = Spinnaker::System::GetInstance();
  camera_list = system->GetCameras();

  if (camera_list.GetSize() < 2) {
    ROS_ERROR("At least two cameras need to be connected. Cameras found = %d",
              camera_list.GetSize());
    exit(-1);
  }

  // Load parameters from ROS
  loadParameters();

  // Connect to cameras
  l_camera = new SpinnakerCamera(l_cam_serial, "left_camera");
  if (!l_camera->connect(camera_list, false)) {
    ROS_ERROR("Could not connect to the camera with the serial number: %s",
              l_cam_serial.c_str());
    exit(-1);
  }

  r_camera = new SpinnakerCamera(r_cam_serial, "right_camera");
  if (!r_camera->connect(camera_list, false)) {
    ROS_ERROR("Could not connect to the camera with the serial number: %s",
              r_cam_serial.c_str());
    exit(-1);
  }

  // Set cameras fps to maximum
  l_camera->setFPS(l_camera->getFPSmax());
  r_camera->setFPS(r_camera->getFPSmax());

  // Initialize frame rate to 1 fps
  frame_rate = new ros::Rate(1.0);

  // Enable camera triggering
  l_camera->enableTriggering(true);
  r_camera->enableTriggering(true);

  // Start acquisition
  if (l_camera->startAcquisition()) std::cout << "Left camera connected\n";
  if (r_camera->startAcquisition()) std::cout << "Right camera connected\n";
  if (!l_camera->getAcquisition() || !r_camera->getAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
    exit(-1);
  }

  // Setup Dynamic Reconfigure Server
  current_config.exposure_time = 0.0;
  current_config.fps = 1.0;
  current_config.gain = 0.0;
  frame_count = 0;
  config_mutex.reset(new std::mutex);
  dynamic_reconfigure::Server<
      spinnaker_driver_ros::stereoCameraParametersConfig>::CallbackType
      config_cb =
          boost::bind(&StereoCameraManagerNodelet::dynamicReconfigureCallback,
                      this, _1, _2);
  config_server.setCallback(config_cb);

  // Setup Publishers
  l_image_pub = image_t.advertise("left_camera/image_raw", 1);
  l_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("left_camera/camera_info", 1);

  r_image_pub = image_t.advertise("right_camera/image_raw", 1);
  r_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("right_camera/camera_info", 1);

  cam_exp_pub = nh.advertise<std_msgs::Float32>("camera_exposure", 1);
  cam_gain_pub = nh.advertise<std_msgs::Float32>("camera_gain", 1);

  frame_grab_worker =
      std::thread(&StereoCameraManagerNodelet::publishImagesSync, this);
}

StereoCameraManagerNodelet::~StereoCameraManagerNodelet() {
  frame_grab_worker.join();
  l_camera->disconnect();
  r_camera->disconnect();
  delete l_camera, r_camera;
  camera_list.Clear();
  system->ReleaseInstance();
}

void StereoCameraManagerNodelet::loadParameters() {
  ros::NodeHandle nh_lcl("~");

  int l_serial, r_serial;
  std::string l_camera_info_file, r_camera_info_file;

  nh_lcl.param("left_camera_serial", l_serial, 01234567);
  nh_lcl.param("right_camera_serial", r_serial, 01234567);
  nh_lcl.param(
      "left_camera_info_url", l_camera_info_file,
      std::string("package://spinnaker_driver_ros/cfg/left_camera.yaml"));
  nh_lcl.param(
      "right_camera_info_url", r_camera_info_file,
      std::string("package://spinnaker_driver_ros/cfg/right_camera.yaml"));

  l_cam_serial = std::to_string(l_serial);
  r_cam_serial = std::to_string(r_serial);

  camera_info_manager::CameraInfoManager cam_manager(nh_lcl);
  cam_manager.setCameraName("left_camera");

  if (cam_manager.loadCameraInfo(l_camera_info_file)) {
    l_cam_info = cam_manager.getCameraInfo();
    l_cam_info_resized = l_cam_info;
  } else
    ROS_ERROR("Could not find left camera calibration file");

  cam_manager.setCameraName("right_camera");

  if (cam_manager.loadCameraInfo(r_camera_info_file)) {
    r_cam_info = cam_manager.getCameraInfo();
    r_cam_info_resized = r_cam_info;
  } else
    ROS_ERROR("Could not find right camera calibration file");
}

void StereoCameraManagerNodelet::publishImagesSync() {
  // Image handles
  cv::Mat l_cap, r_cap;
  ros::Time l_time, r_time;
  while (ros::ok()) {
    std::future<bool> l_image_grab, r_image_grab;
    {  // For async
      std::lock_guard<std::mutex> config_guard(*config_mutex);

      // Send SW trigger to the cameras
      l_camera->softwareTrigger();
      r_camera->softwareTrigger();

      l_image_grab = std::async(std::launch::async, &SpinnakerCamera::grabFrame,
                                l_camera, std::ref(l_cap), 1000);

      r_image_grab = std::async(std::launch::async, &SpinnakerCamera::grabFrame,
                                r_camera, std::ref(r_cap), 1000);
    }

    if (l_image_grab.get() & r_image_grab.get()) {
      ros::Time avg_time = ros::Time::now();

      if (resize_images) {
        cv::Mat l_cap_resized, r_cap_resized;
        cv::resize(l_cap, l_cap_resized, cv::Size(), 1.0 / resize_factor,
                   1.0 / resize_factor, cv::INTER_LINEAR);
        cv::resize(r_cap, r_cap_resized, cv::Size(), 1.0 / resize_factor,
                   1.0 / resize_factor, cv::INTER_LINEAR);

        l_image_pub.publish(toROSImageMsg(l_cap_resized, avg_time));
        r_image_pub.publish(toROSImageMsg(r_cap_resized, avg_time));
        l_cam_info_resized.header.stamp = avg_time;
        l_cam_info_resized.header.seq = frame_count;
        r_cam_info_resized.header.stamp = avg_time;
        r_cam_info_resized.header.seq = frame_count;
        l_cam_info_pub.publish(l_cam_info_resized);
        r_cam_info_pub.publish(r_cam_info_resized);
      } else {
        l_image_pub.publish(toROSImageMsg(l_cap, avg_time));
        r_image_pub.publish(toROSImageMsg(r_cap, avg_time));
        l_cam_info.header.stamp = avg_time;
        l_cam_info.header.seq = frame_count;
        r_cam_info.header.stamp = avg_time;
        r_cam_info.header.seq = frame_count;
        l_cam_info_pub.publish(l_cam_info);
        r_cam_info_pub.publish(r_cam_info);
      }
      frame_count++;
    }
    frame_rate->sleep();
  }
}

void StereoCameraManagerNodelet::dynamicReconfigureCallback(
    spinnaker_driver_ros::stereoCameraParametersConfig& config,
    uint32_t level) {
  resize_factor = config.resize_factor;
  resize_images = config.resize_images;

  if (resize_images) {
    l_cam_info_resized.K[0] = l_cam_info.K[0] / resize_factor;
    l_cam_info_resized.K[2] = l_cam_info.K[2] / resize_factor;
    l_cam_info_resized.K[4] = l_cam_info.K[4] / resize_factor;
    l_cam_info_resized.K[5] = l_cam_info.K[5] / resize_factor;

    r_cam_info_resized.K[0] = r_cam_info.K[0] / resize_factor;
    r_cam_info_resized.K[2] = r_cam_info.K[2] / resize_factor;
    r_cam_info_resized.K[4] = r_cam_info.K[4] / resize_factor;
    r_cam_info_resized.K[5] = r_cam_info.K[5] / resize_factor;
  }

  // Update exposure time
  if (abs(current_config.exposure_time - config.exposure_time) > 0) {
    std::lock_guard<std::mutex> config_guard(*config_mutex);
    if (l_camera->setExposure(config.exposure_time) &&
        r_camera->setExposure(config.exposure_time)) {
      current_config.exposure_time = config.exposure_time;
    }
  }

  // Update Gain
  if (abs(current_config.gain - config.gain) > 0) {
    std::lock_guard<std::mutex> config_guard(*config_mutex);
    if (l_camera->setGain(config.gain) && r_camera->setGain(config.gain)) {
      current_config.gain = config.gain;
    }
  }

  if (abs(current_config.fps - config.fps) > 0) {
    std::lock_guard<std::mutex> config_guard(*config_mutex);
    frame_rate = new ros::Rate(config.fps);
    current_config.fps = config.fps;
  }
}
}  // namespace spinnaker_driver_ros