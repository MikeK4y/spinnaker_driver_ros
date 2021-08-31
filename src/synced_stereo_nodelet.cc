#include "spinnaker_driver_ros/synced_stereo_nodelet.h"

#include <functional>

// ROS
#include <pluginlib/class_list_macros.h>

#include "cv_bridge/cv_bridge.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include "mavros_msgs/CommandTriggerInterval.h"

PLUGINLIB_EXPORT_CLASS(spinnaker_driver_ros::SyncedStereoNodelet,
                       nodelet::Nodelet);

// Helper Functions
inline std::string withLeadingZeros(uint64_t i, uint64_t max = 7) {
  std::ostringstream ss;
  ss << std::setw(max) << std::setfill('0') << i;
  return ss.str();
}

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

void SyncedStereoNodelet::onInit() {
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

  // Set cameras to hardware triggering
  l_camera->setHardwareTrigger();
  r_camera->setHardwareTrigger();

  // Start acquisition
  if (l_camera->startAcquisition()) std::cout << "Left camera connected\n";
  if (r_camera->startAcquisition()) std::cout << "Right camera connected\n";
  if (!l_camera->getAcquisition() || !r_camera->getAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
    exit(-1);
  }

  // Setup Services
  pixhawk_trigger_ctrl = nh.serviceClient<mavros_msgs::CommandTriggerControl>(
      "/mavros/cmd/trigger_control");
  pixhawk_trigger_config =
      nh.serviceClient<mavros_msgs::CommandTriggerInterval>(
          "/mavros/cmd/trigger_interval");

  // Until I figure out how to call the destructor make sure the trigger is off
  // and reset
  triggerControl(false, true);

  // Setup Publishers
  l_image_pub = image_t.advertise("left_camera/image_raw", 1);
  l_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("left_camera/camera_info", 1);

  r_image_pub = image_t.advertise("right_camera/image_raw", 1);
  r_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("right_camera/camera_info", 1);

  // Setup Subscribers
  trigger_time_stamp_sub =
      nh.subscribe("/mavros/cam_imu_sync/cam_imu_stamp", 100,
                   &SyncedStereoNodelet::triggerStampCallback, this);

  // Configure cameras and trigger
  if (fps > 1.0 / (1.0e-6 * exp + 0.014)) {
    fps = floor(1.0 / (1.0e-6 * exp + 0.014));
    ROS_WARN("Selected frame rate is too high. Frame rate will be set to %f",
             fps);
  }
  l_camera->configure(exp, gain, l_camera->getFPSmax());
  r_camera->configure(exp, gain, r_camera->getFPSmax());
  triggerConfig(fps);

  // Initialize frame capturing
  frame_count = 0;
  saved_frame_count = 0;
  if (save_rate > 0.0) {
    startTime = ros::Time::now();
    std::string image_list_file_name = path_to_images + "_image_list.csv";
    image_list_file.open(image_list_file_name);
    if (image_list_file.is_open()) {
      image_list_file << "Count,Filename_0,Filename_1,Time_0,Time_1\n";
    } else {
      ROS_ERROR("Could not open the image list file");
      exit(-1);
    }
  }

  frame_grab_worker =
      std::thread(&SyncedStereoNodelet::publishImagesSync, this);
}

SyncedStereoNodelet::~SyncedStereoNodelet() {
  triggerControl(false, true);
  frame_grab_worker.join();
  if (save_rate > 0.0) image_list_file.close();
  delete l_camera, r_camera;
  camera_list.Clear();
  system->ReleaseInstance();
}

void SyncedStereoNodelet::loadParameters() {
  ros::NodeHandle nh_lcl("~");

  int l_serial, r_serial;
  std::string l_camera_info_file, r_camera_info_file;

  nh_lcl.param("frame_rate", fps, 1.0);
  nh_lcl.param("exposure_time", exp, 1.0);
  nh_lcl.param("gain", gain, 0.0);
  nh_lcl.param("save_rate", save_rate, 0.0);
  nh_lcl.param("resize_factor", resize_factor, 1.0);

  save_percent = save_rate == 0.0 ? 0.0 : uint32_t(1.0 / save_rate);
  resize_factor = resize_factor < 1.0 ? 1.0 : resize_factor;
  buffer_size = size_t(2.0 * fps);

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

void SyncedStereoNodelet::publishImagesSync() {
  // Image handles
  cv::Mat l_cap, r_cap;
  ros::Time l_time, r_time;
  std::string l_file_path, r_file_path;
  // Start triggering
  if (triggerControl(true, true)) {
    while (ros::ok()) {
      bool save_this_frame = false;
      if (save_rate > 0.0) {
        save_this_frame = !bool(frame_count % save_percent);

        if (save_this_frame) {
          l_file_path = path_to_images + "_0_" +
                        withLeadingZeros(saved_frame_count) + ".tif";
          r_file_path = path_to_images + "_1_" +
                        withLeadingZeros(saved_frame_count) + ".tif";
        }
      }

      std::future<bool> l_image_grab, r_image_grab;
      {  // For async
        l_image_grab = std::async(
            std::launch::async, &SpinnakerCamera::grabFrame, l_camera,
            std::ref(l_cap), std::ref(l_file_path), 1000, save_this_frame);

        r_image_grab = std::async(
            std::launch::async, &SpinnakerCamera::grabFrame, r_camera,
            std::ref(r_cap), std::ref(r_file_path), 1000, save_this_frame);
      }

      if (l_image_grab.get() & r_image_grab.get()) {
        // Spin to make sure the trigger callback is up to date
        ros::spinOnce();
        ros::Time timestamp;
        if (getTimestamp(frame_count, timestamp)) {
          if (resize_factor > 1.0) {
            cv::Mat l_cap_resized, r_cap_resized;
            cv::resize(l_cap, l_cap_resized, cv::Size(), 1.0 / resize_factor,
                       1.0 / resize_factor, cv::INTER_LINEAR);
            cv::resize(r_cap, r_cap_resized, cv::Size(), 1.0 / resize_factor,
                       1.0 / resize_factor, cv::INTER_LINEAR);

            l_image_pub.publish(
                toROSImageMsg(l_cap_resized, frame_count, timestamp));
            r_image_pub.publish(
                toROSImageMsg(r_cap_resized, frame_count, timestamp));
            l_cam_info_resized.header.stamp = timestamp;
            l_cam_info_resized.header.seq = frame_count;
            r_cam_info_resized.header.stamp = timestamp;
            r_cam_info_resized.header.seq = frame_count;
            l_cam_info_pub.publish(l_cam_info_resized);
            r_cam_info_pub.publish(r_cam_info_resized);
          } else {
            l_image_pub.publish(toROSImageMsg(l_cap, frame_count, timestamp));
            r_image_pub.publish(toROSImageMsg(r_cap, frame_count, timestamp));
            l_cam_info.header.stamp = timestamp;
            l_cam_info.header.seq = frame_count;
            r_cam_info.header.stamp = timestamp;
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
        } else {
          ROS_ERROR("Could not find the timestamp for the frame");
        }
        frame_count++;
      }
    }
  } else {
    ROS_ERROR("Unable to start the trigger");
  }
}

bool SyncedStereoNodelet::triggerControl(bool enable, bool reset) {
  mavros_msgs::CommandTriggerControl srv;
  srv.request.trigger_enable = enable;
  srv.request.sequence_reset = reset;

  pixhawk_trigger_ctrl.call(srv);
  return srv.response.success;
}

bool SyncedStereoNodelet::triggerConfig(double fps) {
  mavros_msgs::CommandTriggerInterval srv;
  srv.request.cycle_time = 1000.0 / fps;

  pixhawk_trigger_config.call(srv);
  return srv.response.success;
}

void SyncedStereoNodelet::triggerStampCallback(
    const mavros_msgs::CamIMUStamp &msg) {
  timestamp_buffer.emplace_back(msg);

  // Don't let the buffer grow over specified size
  if (timestamp_buffer.size() > buffer_size) {
    timestamp_buffer.erase(timestamp_buffer.begin());
  }
}

bool SyncedStereoNodelet::getTimestamp(uint32_t frame_index,
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