#include "spinnaker_driver_ros/stereo_camera_manager_node.h"

#include <functional>

// ROS
#include "camera_info_manager/camera_info_manager.h"

inline std::string withLeadingZeros(uint64_t i, uint64_t max = 7) {
  std::ostringstream ss;
  ss << std::setw(max) << std::setfill('0') << i;
  return ss.str();
}

StereoCameraManagerNode::StereoCameraManagerNode(
    ros::NodeHandle &nh, image_transport::ImageTransport &image_t) {
  // Initialize Spinnaker handles
  system = Spinnaker::System::GetInstance();
  camera_list = system->GetCameras();

  if (camera_list.GetSize() < 2) {
    ROS_ERROR("At least two cameras need to be connected. Cameras found = %d",
              camera_list.GetSize());
  }

  // Load parameters from ROS
  loadParameters();

  // Connect to cameras
  l_camera = new SpinnakerCamera(l_cam_serial, "left_camera");
  r_camera = new SpinnakerCamera(r_cam_serial, "right_camera");

  if (!l_camera->connect(camera_list) || !r_camera->connect(camera_list)) {
    ROS_ERROR(
        "Could not connect to the cameras with the provided serial numbers");
  }

  {  // Use async to start capturing as much in sync as possible
    auto l_acq = std::async(std::launch::async,
                            [this] { this->l_camera->startAcquisition(); });
    auto r_acq = std::async(std::launch::async,
                            [this]() { this->r_camera->startAcquisition(); });
  }

  if (l_camera->getAcquisition()) std::cout << "Left camera connected\n";
  if (r_camera->getAcquisition()) std::cout << "Right camera connected\n";
  if (!l_camera->getAcquisition() || !r_camera->getAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
  }

  // Setup Dynamic Reconfigure Server
  config_mutex.reset(new std::mutex);
  dynamic_reconfigure::Server<
      spinnaker_driver_ros::stereoCameraParametersConfig>::CallbackType
      config_cb = boost::bind(
          &StereoCameraManagerNode::dynamicReconfigureCallback, this, _1, _2);
  config_server.setCallback(config_cb);

  // Initialize frame capturing
  save_images = true;
  frame_count = 0;
  startTime = ros::Time::now();
  std::string image_list_file_name = path_to_images + "_image_list.csv";
  image_list_file.open(image_list_file_name);
  if (image_list_file.is_open()) {
    image_list_file << "Count,Filename_0,Filename_1,Time_0,Time_1\n";
  } else {
    ROS_ERROR("Could not open the image list file");
  }

  // Setup Publishers
  image_transport::Publisher l_image_pub =
      image_t.advertise("left_camera/image_raw", 1);
  // ros::Publisher l_cam_info_pub =
  //     nh.advertise<sensor_msgs::CameraInfo>("left_camera/camera_info", 1);

  image_transport::Publisher r_image_pub =
      image_t.advertise("right_camera/image_raw", 1);
  // ros::Publisher r_cam_info_pub =
  //     nh.advertise<sensor_msgs::CameraInfo>("right_camera/camera_info", 1);

  frame_grab_worker = std::thread(
      &StereoCameraManagerNode::publishImagesSync, this, std::ref(*l_camera),
      std::ref(*r_camera), l_image_pub, r_image_pub);
}

StereoCameraManagerNode::~StereoCameraManagerNode() {
  image_list_file.close();
  delete l_camera, r_camera;
  camera_list.Clear();
  system->ReleaseInstance();
}

void StereoCameraManagerNode::loadParameters() {
  ros::NodeHandle nh_lcl("~");

  int l_serial, r_serial;

  nh_lcl.param("left_camera_serial", l_serial, 01234567);
  nh_lcl.param("right_camera_serial", r_serial, 01234567);

  l_cam_serial = std::to_string(l_serial);
  r_cam_serial = std::to_string(r_serial);

  nh_lcl.param("path_to_images", path_to_images, std::string("/tmp"));
}

void StereoCameraManagerNode::publishImagesSync(
    SpinnakerCamera &left_camera, SpinnakerCamera &right_camera,
    image_transport::Publisher left_image_pub,
    image_transport::Publisher right_image_pub) {
  // Image handles
  sensor_msgs::Image left_cap, right_cap;
  while (ros::ok()) {
    std::string left_file_path =
        path_to_images + "_0_" + withLeadingZeros(frame_count) + ".tiff";
    std::string right_file_path =
        path_to_images + "_1_" + withLeadingZeros(frame_count) + ".tiff";
    {  // For async
      std::lock_guard<std::mutex> config_guard(*config_mutex);
      auto l_image_grab =
          std::async(std::launch::async, [this, &left_cap, &left_file_path] {
            this->l_camera->grabFrame(left_cap, left_file_path, save_images);
          });

      auto r_image_grab =
          std::async(std::launch::async, [this, &right_cap, &right_file_path] {
            this->r_camera->grabFrame(right_cap, right_file_path, save_images);
          });
    }
    left_image_pub.publish(left_cap);
    right_image_pub.publish(right_cap);

    if (save_images) {
      ros::Duration t_l = left_cap.header.stamp - startTime;
      ros::Duration t_r = right_cap.header.stamp - startTime;

      image_list_file << frame_count << ',' << left_file_path << ','
                      << right_file_path << ',' << t_l.toSec() << ','
                      << t_r.toSec() << '\n';
      frame_count++;
    }
  }
}

void StereoCameraManagerNode::dynamicReconfigureCallback(
    spinnaker_driver_ros::stereoCameraParametersConfig &config,
    uint32_t level) {
  std::lock_guard<std::mutex> config_guard(*config_mutex);
  l_camera->configure(config.exposure_time, config.gain, config.fps);
  r_camera->configure(config.exposure_time, config.gain, config.fps);
}