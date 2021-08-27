#include "spinnaker_driver_ros/stereo_camera_manager_nodelet.h"

#include <functional>

// ROS
#include <pluginlib/class_list_macros.h>

#include "cv_bridge/cv_bridge.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include "mavros_msgs/CommandTriggerInterval.h"

PLUGINLIB_EXPORT_CLASS(spinnaker_driver_ros::StereoCameraManagerNodelet,
                       nodelet::Nodelet);

// Helper Functions
inline std::string withLeadingZeros(uint64_t i, uint64_t max = 7) {
  std::ostringstream ss;
  ss << std::setw(max) << std::setfill('0') << i;
  return ss.str();
}

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
  r_camera = new SpinnakerCamera(r_cam_serial, "right_camera");

  if (!l_camera->connect(camera_list) || !r_camera->connect(camera_list)) {
    ROS_ERROR(
        "Could not connect to the cameras with the provided serial numbers");
    exit(-1);
  }

  std::future<bool> l_acq, r_acq;
  {  // Use async to start capturing as much in sync as possible
    l_acq = std::async(std::launch::async, &SpinnakerCamera::startAcquisition,
                       l_camera);
    r_acq = std::async(std::launch::async, &SpinnakerCamera::startAcquisition,
                       r_camera);
  }

  if (l_acq.get()) std::cout << "Left camera connected\n";
  if (r_acq.get()) std::cout << "Right camera connected\n";
  if (!l_camera->getAcquisition() || !r_camera->getAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
    exit(-1);
  }

  // Setup Dynamic Reconfigure Server
  current_config.exposure_time = 0.0;
  current_config.fps = 0.0;
  current_config.gain = 0.0;
  frame_count = 0;
  saved_frame_count = 0;
  is_hardware_trigger = false;
  config_mutex.reset(new std::mutex);
  dynamic_reconfigure::Server<
      spinnaker_driver_ros::stereoCameraParametersConfig>::CallbackType
      config_cb =
          boost::bind(&StereoCameraManagerNodelet::dynamicReconfigureCallback,
                      this, _1, _2);
  config_server.setCallback(config_cb);

  // Initialize frame capturing
  startTime = ros::Time::now();
  std::string image_list_file_name = path_to_images + "_image_list.csv";
  image_list_file.open(image_list_file_name);
  if (image_list_file.is_open()) {
    image_list_file << "Count,Filename_0,Filename_1,Time_0,Time_1\n";
  } else {
    ROS_ERROR("Could not open the image list file");
    exit(-1);
  }

  // Setup Services
  pixhawk_trigger_ctrl = nh.serviceClient<mavros_msgs::CommandTriggerControl>(
      "/mavros/cmd/trigger_control");
  pixhawk_trigger_config =
      nh.serviceClient<mavros_msgs::CommandTriggerInterval>(
          "/mavros/cmd/trigger_interval");

  // Setup Publishers
  l_image_pub = image_t.advertise("left_camera/image_raw", 1);
  l_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("left_camera/camera_info", 1);

  r_image_pub = image_t.advertise("right_camera/image_raw", 1);
  r_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("right_camera/camera_info", 1);

  frame_grab_worker =
      std::thread(&StereoCameraManagerNodelet::publishImagesSync, this);
}

StereoCameraManagerNodelet::~StereoCameraManagerNodelet() {
  frame_grab_worker.join();
  image_list_file.close();
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

  nh_lcl.param("path_to_images", path_to_images, std::string("/tmp"));
}

void StereoCameraManagerNodelet::publishImagesSync() {
  // Image handles
  cv::Mat l_cap, r_cap;
  ros::Time l_time, r_time;
  std::string l_file_path, r_file_path;
  while (ros::ok()) {
    bool save_this_frame = save_images & !bool(frame_count % save_percent);
    if (save_this_frame) {
      l_file_path =
          path_to_images + "_0_" + withLeadingZeros(saved_frame_count) + ".tif";
      r_file_path =
          path_to_images + "_1_" + withLeadingZeros(saved_frame_count) + ".tif";
    }
    std::future<bool> l_image_grab, r_image_grab;
    {  // For async
      std::lock_guard<std::mutex> config_guard(*config_mutex);
      l_image_grab = std::async(std::launch::async, &SpinnakerCamera::grabFrame,
                                l_camera, std::ref(l_cap),
                                std::ref(l_file_path), 1000, save_this_frame);

      r_image_grab = std::async(std::launch::async, &SpinnakerCamera::grabFrame,
                                r_camera, std::ref(r_cap),
                                std::ref(r_file_path), 1000, save_this_frame);
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

      if (save_this_frame) {
        ros::Duration t_l = l_time - startTime;
        ros::Duration t_r = r_time - startTime;

        image_list_file << saved_frame_count << ',' << l_file_path << ','
                        << r_file_path << ',' << t_l.toSec() << ','
                        << t_r.toSec() << '\n';
        saved_frame_count++;
      }
      frame_count++;
    }
  }
}

bool StereoCameraManagerNodelet::triggerControl(bool enable) {
  mavros_msgs::CommandTriggerControl srv;
  srv.request.trigger_enable = enable;

  pixhawk_trigger_ctrl.call(srv);
  return srv.response.success;
}

bool StereoCameraManagerNodelet::triggerConfig(double fps) {
  mavros_msgs::CommandTriggerInterval srv;
  srv.request.cycle_time = 1000.0 / fps;

  pixhawk_trigger_config.call(srv);
  return srv.response.success;
}

void StereoCameraManagerNodelet::dynamicReconfigureCallback(
    spinnaker_driver_ros::stereoCameraParametersConfig& config,
    uint32_t level) {
  save_percent = uint64_t(1.0 / config.save_percent);
  save_images = config.save_images;
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

  // In hardware triggering the frame rate should be controlled by the
  // triggering hardware
  if (config.hardware_trigger && !is_hardware_trigger) {
    l_camera->setHardwareTrigger();
    r_camera->setHardwareTrigger();

    // Set fps to maximum allowed so that the software won't limit the
    // hardware
    std::lock_guard<std::mutex> config_guard(*config_mutex);
    l_camera->setFPS(l_camera->getFPSmax());
    r_camera->setFPS(r_camera->getFPSmax());

    if (triggerConfig(current_config.fps)) {
      if (triggerControl(true)) is_hardware_trigger = true;
    }

    if (!is_hardware_trigger) {
      l_camera->setFPS(current_config.fps);
      r_camera->setFPS(current_config.fps);
    }

  } else if (is_hardware_trigger && !config.hardware_trigger) {
    if (triggerControl(false)) {
      l_camera->setContinuousCapture();
      r_camera->setContinuousCapture();

      // Set fps to the value from the config server
      std::lock_guard<std::mutex> config_guard(*config_mutex);
      l_camera->setFPS(current_config.fps);
      r_camera->setFPS(current_config.fps);

      is_hardware_trigger = false;
    }
  }

  // Check if the camera settings have changed
  double diff = abs(current_config.gain - config.gain) +
                abs(current_config.fps - config.fps) +
                abs(current_config.exposure_time - config.exposure_time);

  if (diff > 0) {
    std::lock_guard<std::mutex> config_guard(*config_mutex);
    if (!is_hardware_trigger) {
      l_camera->configure(config.exposure_time, config.gain, config.fps);
      r_camera->configure(config.exposure_time, config.gain, config.fps);
    } else {
      l_camera->setExposure(config.exposure_time);
      r_camera->setExposure(config.exposure_time);
      l_camera->setGain(config.gain);
      r_camera->setGain(config.gain);

      // To change the trigger rate you must disable the trigger, change the
      // rate and then enable it again
      if (triggerControl(false)) {
        if (triggerConfig(config.fps)) {
          std::cout << "Frame rate set to " << config.fps << "fps\n";
        } else {
          std::cout << "Could not set the frame rate to " << config.fps
                    << "fps\n";
        }
      }
      if (!triggerControl(true)) {
        std::cout << "Failled to restart trigger after rate change\n";
      }
    }

    current_config.gain = config.gain;
    current_config.fps = config.fps;
    current_config.exposure_time = config.exposure_time;
  }
}
}  // namespace spinnaker_driver_ros