#include "spinnaker_driver_ros/stereo_camera_manager_node.h"

// ROS
#include "camera_info_manager/camera_info_manager.h"

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
  l_camera = new SpinnakerCamera(l_cam_serial);
  r_camera = new SpinnakerCamera(r_cam_serial);

  if (!l_camera->connect(camera_list) || !r_camera->connect(camera_list)) {
    ROS_ERROR(
        "Could not connect to the cameras with the provided serial numbers");
  }

  // Initialize cameras
  l_camera->configure(1000.0, 10.0);
  r_camera->configure(1000.0, 10.0);

  if (!l_camera->startAcquisition() || !r_camera->startAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
  }

  // Setup Dynamic Reconfigure Server
  dynamic_reconfigure::Server<
      spinnaker_driver_ros::stereoCameraParametersConfig>::CallbackType
      config_cb = boost::bind(
          &StereoCameraManagerNode::dynamicReconfigureCallback, this, _1, _2);
  config_server.setCallback(config_cb);

  // Setup Publishers
  image_transport::Publisher l_image_pub =
      image_t.advertise("left_camera/image_raw", 1);
  ros::Publisher l_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("left_camera/camera_info", 1);

  image_transport::Publisher r_image_pub =
      image_t.advertise("right_camera/image_raw", 1);
  ros::Publisher r_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("right_camera/camera_info", 1);

  l_cam_worker = std::thread(&StereoCameraManagerNode::publishImage, this,
                             std::ref(*l_camera), l_image_pub);
  r_cam_worker = std::thread(&StereoCameraManagerNode::publishImage, this,
                             std::ref(*r_camera), r_image_pub);
}

StereoCameraManagerNode::~StereoCameraManagerNode() {
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
}

void StereoCameraManagerNode::publishImage(
    SpinnakerCamera &camera, image_transport::Publisher image_pub) {
  while (ros::ok()) {
    cv::Mat cap;
    uint64_t time_stamp;
    if (camera.grabFrame(cap, time_stamp)) {
      cv_bridge::CvImage ros_mat;
      sensor_msgs::Image ros_image;

      ros_mat.image = cap;
      ros_mat.header.stamp.sec = time_stamp;
      ros_mat.header.stamp.nsec = time_stamp * 1e-9;
      ros_mat.encoding = "mono8";
      // TODO: Add header.frame_id

      ros_mat.toImageMsg(ros_image);

      image_pub.publish(ros_image);
    } else {
      ROS_WARN("Did not received a frame! Trying again");
    }
  }
}

void StereoCameraManagerNode::dynamicReconfigureCallback(
    spinnaker_driver_ros::stereoCameraParametersConfig &config,
    uint32_t level) {
  l_camera->configure(config.exposure_time, config.fps);
  r_camera->configure(config.exposure_time, config.fps);
}