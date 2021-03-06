#include "spinnaker_driver_ros/hardware_synced_nodelet.h"

#include <functional>

// ROS
#include <pluginlib/class_list_macros.h>

#include "cv_bridge/cv_bridge.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include "mavros_msgs/CommandTriggerInterval.h"
#include "std_msgs/Float32.h"

PLUGINLIB_EXPORT_CLASS(spinnaker_driver_ros::HardwareSyncedNodelet,
                       nodelet::Nodelet);

// Helper Functions
inline sensor_msgs::Image toROSImageMsg(cv::Mat frame, uint32_t sequence,
                                        ros::Time timestamp) {
  cv_bridge::CvImage ros_mat;
  sensor_msgs::Image ros_image;

  ros_mat.image = frame;
  ros_mat.header.seq = sequence;
  ros_mat.header.stamp = timestamp;
  ros_mat.encoding = "mono8";

  ros_mat.toImageMsg(ros_image);

  return ros_image;
}

namespace spinnaker_driver_ros {

void HardwareSyncedNodelet::onInit() {
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
  if (!l_camera->connect(camera_list, true)) {
    ROS_ERROR("Could not connect to the camera with the serial number: %s",
              l_cam_serial.c_str());
    exit(-1);
  }

  r_camera = new SpinnakerCamera(r_cam_serial, "right_camera");
  if (!r_camera->connect(camera_list, true)) {
    ROS_ERROR("Could not connect to the camera with the serial number: %s",
              r_cam_serial.c_str());
    exit(-1);
  }

  // Configure cameras
  l_camera->configure(exp, gain, l_camera->getFPSmax());
  r_camera->configure(exp, gain, r_camera->getFPSmax());

  // Make sure we can start acquisition
  if (l_camera->startAcquisition()) std::cout << "Left camera connected\n";
  if (r_camera->startAcquisition()) std::cout << "Right camera connected\n";
  if (!l_camera->getAcquisition() || !r_camera->getAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
    exit(-1);
  }

  l_camera->stopAcquisition();
  r_camera->stopAcquisition();

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

  cam_exp_pub = nh.advertise<std_msgs::Float32>("camera_exposure", 1);
  cam_gain_pub = nh.advertise<std_msgs::Float32>("camera_gain", 1);

  // Setup Subscribers
  trigger_time_stamp_sub =
      nh.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 100,
                   &HardwareSyncedNodelet::triggerStampCallback, this);

  // Configure cameras and trigger
  if (fps > 1.0 / (1.0e-6 * exp + 0.014)) {
    fps = floor(1.0 / (1.0e-6 * exp + 0.014));
    ROS_WARN("Selected frame rate is too high. Frame rate will be set to %f",
             fps);
  }
  triggerConfig(fps);
  frame_count = 0;

  frame_grab_worker =
      std::thread(&HardwareSyncedNodelet::publishImagesSync, this);
}

HardwareSyncedNodelet::~HardwareSyncedNodelet() {
  triggerControl(false, true);
  frame_grab_worker.join();
  l_camera->disconnect();
  r_camera->disconnect();
  delete l_camera, r_camera;
  camera_list.Clear();
  system->ReleaseInstance();
}

void HardwareSyncedNodelet::loadParameters() {
  ros::NodeHandle nh_lcl("~");

  nh_lcl.param("frame_rate", fps, 1.0);
  nh_lcl.param("exposure_time", exp, 1.0);
  nh_lcl.param("gain", gain, 0.0);

  buffer_size = size_t(2.0 * fps);

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

  if (cam_manager.loadCameraInfo(l_camera_info_file))
    l_cam_info = cam_manager.getCameraInfo();
  else
    ROS_ERROR("Could not find left camera calibration file");

  cam_manager.setCameraName("right_camera");

  if (cam_manager.loadCameraInfo(r_camera_info_file))
    r_cam_info = cam_manager.getCameraInfo();
  else
    ROS_ERROR("Could not find right camera calibration file");
}

void HardwareSyncedNodelet::publishImagesSync() {
  // Make sure trigger is reset and stopped before starting the cameras
  while (!triggerControl(false, true)) {
    ROS_WARN("Cannot reset HW Trigger");
  }
  ROS_INFO("HW Trigger was reset");

  // Enable camera triggering
  l_camera->enableTriggering(true);
  r_camera->enableTriggering(true);

  // Start Acquisition
  l_camera->startAcquisition();
  r_camera->startAcquisition();

  std_msgs::Float32 exp_msg, gain_msg;
  cv::Mat l_cap, r_cap;

  // Start triggering
  if (triggerControl(true, true)) {
    while (ros::ok()) {
      std::future<bool> l_image_grab, r_image_grab;
      {
        l_image_grab =
            std::async(std::launch::async, &SpinnakerCamera::grabFrame,
                       l_camera, std::ref(l_cap), 1000);

        r_image_grab =
            std::async(std::launch::async, &SpinnakerCamera::grabFrame,
                       r_camera, std::ref(r_cap), 1000);
      }

      if (l_image_grab.get() & r_image_grab.get()) {
        // Spin to make sure the trigger callback is up to date
        ros::spinOnce();
        ros::Time timestamp;
        if (getTimestamp(frame_count, timestamp)) {
          l_image_pub.publish(toROSImageMsg(l_cap, frame_count, timestamp));
          r_image_pub.publish(toROSImageMsg(r_cap, frame_count, timestamp));
          l_cam_info.header.stamp = timestamp;
          l_cam_info.header.seq = frame_count;
          r_cam_info.header.stamp = timestamp;
          r_cam_info.header.seq = frame_count;
          l_cam_info_pub.publish(l_cam_info);
          r_cam_info_pub.publish(r_cam_info);

          // Publish current exposure
          exp_msg.data = exp;
          cam_exp_pub.publish(exp_msg);

          // Publish current gain;
          gain_msg.data = gain;
          cam_gain_pub.publish(gain_msg);

        } else {
          ROS_ERROR("Dropping frame! Could not find the timestamp");
        }
        frame_count++;
      }
    }
  } else {
    ROS_ERROR("Unable to start the trigger");
  }
}

bool HardwareSyncedNodelet::triggerControl(bool enable, bool reset) {
  mavros_msgs::CommandTriggerControl srv;
  srv.request.trigger_enable = enable;
  srv.request.sequence_reset = reset;

  pixhawk_trigger_ctrl.call(srv);
  return srv.response.success;
}

bool HardwareSyncedNodelet::triggerConfig(double fps) {
  mavros_msgs::CommandTriggerInterval srv;
  srv.request.cycle_time = 1000.0 / fps;

  pixhawk_trigger_config.call(srv);
  return srv.response.success;
}

void HardwareSyncedNodelet::triggerStampCallback(
    const mavros_msgs::CamIMUStamp &msg) {
  timestamp_buffer.emplace_back(msg);

  // Don't let the buffer grow over specified size
  if (timestamp_buffer.size() > buffer_size) {
    timestamp_buffer.erase(timestamp_buffer.begin());
  }
}

bool HardwareSyncedNodelet::getTimestamp(uint32_t frame_index,
                                         ros::Time &timestamp) {
  if (timestamp_buffer.size() < 1) {
    return false;
  } else {
    bool stamp_found = false;
    size_t buffer_index = 0;
    while (buffer_index < timestamp_buffer.size()) {
      if (timestamp_buffer[buffer_index].frame_seq_id == frame_index) {
        timestamp = timestamp_buffer[buffer_index].frame_stamp;
        stamp_found = true;
        break;
      }
      buffer_index++;
    }
    return stamp_found;
  }
}
}  // namespace spinnaker_driver_ros